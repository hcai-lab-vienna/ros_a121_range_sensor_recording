#!/usr/bin/env python3
# pyright: reportPrivateImportUsage=false, reportArgumentType=false, reportCallIssue=false
# pyright: reportAttributeAccessIssue=false

from time import sleep
from pathlib import Path
import os

import acconeer.exptool as et
from acconeer.exptool import a121
from acconeer.exptool._core.communication.links.buffered_link import LinkError
from acconeer.exptool.a121._core.entities.configs.config_enums import (
    PRF,
    IdleState,
    Profile,
)
from serial.serialutil import SerialException


def main():
    parser = a121.ExampleArgumentParser()
    parser.add_argument("--output-file", required=False, default="out.h5")
    args = parser.parse_args()
    et.utils.config_logging(args)

    if Path(args.output_file).exists():
        os.remove(args.output_file)

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
        start_point=80,
        num_points=40,
        step_length=8,
        hwaas=8,
        profile=Profile.PROFILE_3,
        receiver_gain=16,
        prf=PRF.PRF_15_6_MHz,
        enable_tx=True,
        enable_loopback=False,
        phase_enhancement=False,
        iq_imbalance_compensation=False,
    )
    sensor_config = a121.SensorConfig(
        sweeps_per_frame=32,
        sweep_rate=None,
        frame_rate=None,
        inter_sweep_idle_state=IdleState.READY,
        inter_frame_idle_state=IdleState.DEEP_SLEEP,
        continuous_sweep_mode=False,
        double_buffering=False,
        subsweeps=[subsweep_config],
    )
    session_config = a121.SessionConfig([{sensor_id: sensor_config}], extended=True)

    client.setup_session(session_config)

    with a121.H5Recorder(args.output_file, client):
        client.start_session()
        interrupt_handler = et.utils.ExampleInterruptHandler()
        print("Press Ctrl-C to end session")
        while not interrupt_handler.got_signal:
            client.get_next()
        print("Disconnecting...")
        client.stop_session()

    client.close()


if __name__ == "__main__":
    main()
