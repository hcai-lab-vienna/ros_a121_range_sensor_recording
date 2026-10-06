#!/usr/bin/env python3
# pyright: reportPrivateImportUsage=false, reportArgumentType=false, reportCallIssue=false
# pyright: reportAttributeAccessIssue=false


import copy
from time import sleep

import acconeer.exptool as et
from acconeer.exptool import a121
from acconeer.exptool._core.communication.links.buffered_link import LinkError
from acconeer.exptool.a121._core.entities.configs.config_enums import (
    PRF,
    IdleState,
)
from acconeer.exptool.a121.algo.distance import (
    Processor,
    ProcessorConfig,
    ProcessorContext,
    ThresholdMethod,
    calculate_bg_noise_std,
)
from serial.serialutil import SerialException


def main():
    parser = a121.ExampleArgumentParser()
    parser.add_argument("--output-file", required=False, default="out.h5")
    args = parser.parse_args()
    et.utils.config_logging(args)

    client = None
    while client is None:
        try:
            client = a121.Client.open(**a121.get_client_args(args))
        except (SerialException, LinkError) as e:
            print(e)
            sleep(1)
        else:
            break

    sensor_id = 1
    subsweep_config = a121.SubsweepConfig(
        start_point=50,
        num_points=50,
        step_length=4,
        hwaas=32,
        profile=a121.Profile.PROFILE_1,
        receiver_gain=16,
        prf=PRF.PRF_15_6_MHz,
        enable_tx=True,
        enable_loopback=False,
        phase_enhancement=True,
        iq_imbalance_compensation=True,
    )
    sensor_config = a121.SensorConfig(
        sweeps_per_frame=1,
        sweep_rate=None,
        frame_rate=None,
        inter_sweep_idle_state=IdleState.READY,
        inter_frame_idle_state=IdleState.DEEP_SLEEP,
        continuous_sweep_mode=False,
        double_buffering=False,
        subsweeps=[subsweep_config],
    )
    session_config = a121.SessionConfig([{sensor_id: sensor_config}], extended=True)

    # Calibrate noise.
    noise_sensor_config = copy.deepcopy(sensor_config)
    for subsweep in noise_sensor_config.subsweeps:
        subsweep.enable_tx = False
    metadata = client.setup_session(noise_sensor_config)
    client.start_session()
    result = client.get_next()
    client.stop_session()
    stds = [
        calculate_bg_noise_std(subframe, subsweep_config)
        for (subframe, subsweep_config) in zip(
            result.subframes, noise_sensor_config.subsweeps
        )
    ]
    distance_context = ProcessorContext(bg_noise_std=stds)
    distance_config = ProcessorConfig(
        threshold_method=ThresholdMethod.CFAR,
        threshold_sensitivity=0.8
    )
    distance_processor = Processor(
        session_config=session_config,
        metadata=metadata,
        processor_config=distance_config,
        context=distance_context,
    )

    client.setup_session(session_config)

    with a121.H5Recorder(args.output_file, client):
        client.start_session()
        interrupt_handler = et.utils.ExampleInterruptHandler()
        print("Press Ctrl-C to end session")
        while not interrupt_handler.got_signal:
            extended_result = client.get_next()
            processed_data = distance_processor.process(extended_result)
            client.get_next()
        print("Disconnecting...")
        client.stop_session()

    client.close()


if __name__ == "__main__":
    main()
