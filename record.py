#!/usr/bin/env python3
# pyright: reportPrivateImportUsage=false
# pyright: reportArgumentType=false
# pyright: reportCallIssue=false
# pyright: reportAttributeAccessIssue=false
# pyright: reportPossiblyUnboundVariable=false

import os
from copy import deepcopy
from pathlib import Path
from sys import exit
from time import sleep, time

import acconeer.exptool as et
import numpy as np
import rclpy
from acconeer.exptool import a121
from acconeer.exptool._core.communication.links.buffered_link import LinkError
from acconeer.exptool.a121._core.entities.configs.config_enums import Profile
from acconeer.exptool.a121.algo.distance import (
    Processor,
    ProcessorConfig,
    ProcessorContext,
    ThresholdMethod,
    calculate_bg_noise_std,
)
from rclpy.node import Node
from sensor_msgs.msg import Range
from serial.serialutil import SerialException
from std_msgs.msg import Header


class A121Node(Node):
    def __init__(self):
        super().__init__("range_node")
        self.range_pub = self.create_publisher(Range, "/range", 10)

        parser = a121.ExampleArgumentParser()
        parser.add_argument("--output-file", required=False, type=str,   default=None) # output file extension is .h5
        parser.add_argument("--bin-size",    required=False, type=int,   default=0   )
        parser.add_argument("--start-point", required=False, type=int,   default=46  )
        parser.add_argument("--num-points",  required=False, type=int,   default=103 )
        parser.add_argument("--step-length", required=False, type=int,   default=4   )
        parser.add_argument("--hwaas",       required=False, type=int,   default=32  )
        parser.add_argument("--sensitivity", required=False, type=float, default=0.8 )
        args = parser.parse_args()
        if args.serial_port is None:
            args.serial_port = "/dev/ttyACM0"
        if args.output_file and Path(args.output_file).exists():
            os.remove(args.output_file)
        self.output_file = args.output_file
        self.bin_size = args.bin_size
        et.utils.config_logging(args)
        self.client = self.force_start_client(args)

        self.subsweep_config = a121.SubsweepConfig(
            start_point=args.start_point,
            num_points=args.num_points,
            step_length=args.step_length,
            hwaas=args.hwaas,
            profile=Profile.PROFILE_1,
            phase_enhancement=True,
            iq_imbalance_compensation=True,
        )
        self.sensor_config = a121.SensorConfig(
            sweeps_per_frame=1, subsweeps=[self.subsweep_config]
        )

        metadata, context = self.calibrate_noise()
        self.distance_config = ProcessorConfig(
            threshold_method=ThresholdMethod.CFAR,
            threshold_sensitivity=args.sensitivity,
        )
        self.distance_processor = Processor(
            sensor_config=self.sensor_config,
            metadata=metadata,
            processor_config=self.distance_config,
            context=context,
        )

        self.client.setup_session(self.sensor_config)
        print("Press Ctrl-C to end session")
        try:
            if self.output_file:
                self.record_and_start()
            else:
                self.start()
        finally:
            print("\nDisconnecting...")
            self.client.stop_session()
        self.client.close()
        rclpy.shutdown()
        exit()

    @staticmethod
    def force_start_client(args):
        while True:
            try:
                sleep(1)
                return a121.Client.open(**a121.get_client_args(args))
            except (SerialException, LinkError) as e:
                print(e)
            except KeyboardInterrupt:
                print("\nAborting...")
                rclpy.shutdown()
                exit()

    def calibrate_noise(self):
        noise_sensor_config = deepcopy(self.sensor_config)
        for subsweep in noise_sensor_config.subsweeps:
            subsweep.enable_tx = False
        metadata = self.client.setup_session(noise_sensor_config)
        self.client.start_session()
        result = self.client.get_next()
        self.client.stop_session()
        context = ProcessorContext(
            bg_noise_std=[
                calculate_bg_noise_std(subframe, subsweep_config)
                for (subframe, subsweep_config) in zip(
                    result.subframes, noise_sensor_config.subsweeps
                )
            ]
        )
        return metadata, context

    def record_and_start(self):
        assert self.output_file
        with a121.H5Recorder(self.output_file, self.client):
            self.start()

    def start(self):
        self.client.start_session()
        start_time = time()
        if self.bin_size > 1:
            history = [np.nan] * self.bin_size
        interrupt_handler = et.utils.ExampleInterruptHandler()

        while not interrupt_handler.got_signal:
            dt = time() - start_time
            header_msg = Header()
            header_msg.stamp.sec = sec = int(dt)
            header_msg.stamp.nanosec = int((dt - sec) * 1e9)
            header_msg.frame_id = "RADAR"
            range_msg = Range()
            range_msg.header = header_msg
            range_msg.radiation_type = 60
            range_msg.field_of_view = 1.03
            # sensor FOV: 53° horizontal, 65° vertical
            # average is used: 59° -> 1.03 rad

            if self.output_file:
                result = deepcopy(self.client.get_next())
            else:
                result = self.client.get_next()
            distances = self.distance_processor.process(result).estimated_distances

            # while binning: only the first distance provided by the sensor is used
            # min, max and var are calculated for the history of the first distances
            if self.bin_size > 1:
                history.pop(0)
                history.append(distances[0] if distances else np.nan)
                mean_value = np.mean(history)
                if not np.isnan(mean_value):
                    range_msg.range = mean_value
                    range_msg.min_range = np.min(history)
                    range_msg.max_range = np.max(history)
                    range_msg.variance = np.var(history)
                    self.range_pub.publish(range_msg)

            # if no binning: min, max and var are calculated by reported distances of the sensor
            # the sensor can provides multiple distances of returns when there are more objects reflecting
            # the first distance is usually the better measurement value and also (usually) the shortest distance
            elif distances: # and self.bin_size <= 1
                range_msg.range = distances[0]
                range_msg.min_range = np.min(distances)
                range_msg.max_range = np.max(distances)
                range_msg.variance = np.var(distances)
                self.range_pub.publish(range_msg)


def main(args=None):
    rclpy.init(args=args)
    node = A121Node()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
