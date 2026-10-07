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
    def __init__(self, sample_size=5):
        super().__init__("range_node")
        self.range_pub = self.create_publisher(Range, "/range", 10)
        self.sample_size = sample_size

        parser = a121.ExampleArgumentParser()
        parser.add_argument("--output-file", required=False, default="out.h5")
        args = parser.parse_args()
        if args.serial_port is None:
            args.serial_port = "/dev/ttyACM0"
        if Path(args.output_file).exists():
            os.remove(args.output_file)
        et.utils.config_logging(args)
        self.output_file = args.output_file

        self.client = self.force_start_client(args)

        self.subsweep_config = a121.SubsweepConfig(
            start_point=46,
            num_points=103,
            step_length=4,
            hwaas=32,
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
            threshold_sensitivity=0.8,
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
            self.start()
        finally:
            print("Disconnecting...")
            self.client.stop_session()
        self.client.close()

    @staticmethod
    def force_start_client(args):
        while True:
            try:
                sleep(1)
                return a121.Client.open(**a121.get_client_args(args))
            except (SerialException, LinkError) as e:
                print(e)
            except KeyboardInterrupt:
                print("Aborting...")
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

    def start(self):
        with a121.H5Recorder(self.output_file, self.client):
            self.client.start_session()
            start_time = time()
            history = [np.nan] * self.sample_size
            interrupt_handler = et.utils.ExampleInterruptHandler()
            while not interrupt_handler.got_signal:
                result = deepcopy(self.client.get_next())
                distances = self.distance_processor.process(result).estimated_distances
                if len(distances) != 0:
                    for d in distances:  # type: ignore
                        history.pop(0)
                        history.append(d)
                else:
                    history.pop(0)
                    history.append(np.nan)
                mean_value = np.mean(history)
                if not np.isnan(mean_value):
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
                    range_msg.min_range = np.min(history)
                    range_msg.max_range = np.max(history)
                    range_msg.range = mean_value
                    range_msg.variance = np.var(history)
                    self.range_pub.publish(range_msg)


def main(args=None):
    rclpy.init(args=args)
    node = A121Node()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()


# def start_a121_recording_without_ros():
#     parser = a121.ExampleArgumentParser()
#     parser.add_argument("--output-file", required=False, default="out.h5")
#     parser.add_argument("--distances-file", required=False, default=None)
#     args = parser.parse_args()
#     if args.serial_port is None:
#         args.serial_port = "/dev/ttyACM0"
#     if args.output_file is not None and Path(args.output_file).exists():
#         os.remove(args.output_file)
#     record_distances_file = args.distances_file is not None
#     et.utils.config_logging(args)
#
#     client = A121Node.force_start_client(args)
#
#     subsweep_config = a121.SubsweepConfig(
#         start_point=46,
#         num_points=103,
#         step_length=4,
#         hwaas=32,
#         profile=Profile.PROFILE_1,
#         phase_enhancement=True,
#         iq_imbalance_compensation=True,
#     )
#     sensor_config = a121.SensorConfig(sweeps_per_frame=1, subsweeps=[subsweep_config])
#
#     noise_sensor_config = deepcopy(sensor_config)
#     for subsweep in noise_sensor_config.subsweeps:
#         subsweep.enable_tx = False
#     metadata = client.setup_session(noise_sensor_config)
#     client.start_session()
#     result = client.get_next()
#     client.stop_session()
#     context = ProcessorContext(
#         bg_noise_std=[
#             calculate_bg_noise_std(subframe, subsweep_config)
#             for (subframe, subsweep_config) in zip(
#                 result.subframes, noise_sensor_config.subsweeps
#             )
#         ]
#     )
#     distance_config = ProcessorConfig(
#         threshold_method=ThresholdMethod.CFAR,
#         threshold_sensitivity=0.8,
#     )
#     distance_processor = Processor(
#         sensor_config=sensor_config,
#         metadata=metadata,
#         processor_config=distance_config,
#         context=context,
#     )
#
#     client.setup_session(sensor_config)
#
#     with a121.H5Recorder(args.output_file, client):
#         try:
#             if record_distances_file:
#                 distances_file = open(args.distances_file, "w")  # noqa: SIM115
#             client.start_session()
#             start_time = time()
#             history = [np.nan] * 3
#             interrupt_handler = et.utils.ExampleInterruptHandler()
#             print("Press Ctrl-C to end session")
#             while not interrupt_handler.got_signal:
#                 result = deepcopy(client.get_next())
#                 distances = distance_processor.process(result).estimated_distances
#                 if len(distances) != 0:
#                     for d in distances:  # type: ignore
#                         d *= 100
#                         history.pop(0)
#                         history.append(d)
#                 else:
#                     history.pop(0)
#                     history.append(np.nan)
#                 mean_value = np.mean(history)
#                 if not np.isnan(mean_value):
#                     dt = time() - start_time
#                     if record_distances_file:
#                         distances_file.write(f"{dt},{mean_value}\n")
#         finally:
#             print("Disconnecting...")
#             if record_distances_file:
#                 distances_file.close()
#             client.stop_session()
#
#     client.close()

# subsweep_config = a121.SubsweepConfig(
#     start_point=80,
#     num_points=40,
#     step_length=8,
#     hwaas=8,
#     profile=Profile.PROFILE_3,
#     receiver_gain=16,
#     prf=PRF.PRF_15_6_MHz,
#     enable_tx=True,
#     enable_loopback=False,
#     phase_enhancement=False,
#     iq_imbalance_compensation=False,
# )
# sensor_config = a121.SensorConfig(
#     sweeps_per_frame=32,
#     sweep_rate=None,
#     frame_rate=None,
#     inter_sweep_idle_state=IdleState.READY,
#     inter_frame_idle_state=IdleState.DEEP_SLEEP,
#     continuous_sweep_mode=False,
#     double_buffering=False,
#     subsweeps=[subsweep_config],
# )
# sensor_id = 1
# session_config = a121.SessionConfig(
#     [
#         {sensor_id: sensor_config}
#     ],
#     extended=True
# )
# client.setup_session(session_config)
