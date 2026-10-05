#!/usr/bin/env python3

from time import sleep

import acconeer.exptool as et
from acconeer.exptool import a121
from acconeer.exptool.a121._core.entities.configs.config_enums import PRF, IdleState


parser = a121.ExampleArgumentParser()
parser.add_argument("--output-file", required=True)
args = parser.parse_args()
et.utils.config_logging(args)

client = None
while client is None:
    try:
        client = a121.Client.open(**a121.get_client_args(args))
    except Exception as e:
        print(e)
    else:
        break

sensor_id = 1
sensor_config = a121.SensorConfig(
    sweeps_per_frame=32,
    sweep_rate=None,
    frame_rate=None,
    inter_sweep_idle_state=IdleState.READY,
    inter_frame_idle_state=IdleState.DEEP_SLEEP,
    continuous_sweep_mode=False,
    double_buffering=False,
    subsweeps=[
        a121.SubsweepConfig(
            start_point=80,
            num_points=40,
            step_length=8,
            hwaas=8,
            profile=3,
            receiver_gain=16,
            prf=PRF.PRF_15_6_MHz,
            enable_tx=True,
            enable_loopback=False,
            phase_enhancement=False,
            iq_imbalance_compensation=False,
        ),
    ],
)

session_config = a121.SessionConfig(
    [
        {
            sensor_id: sensor_config,
        }
    ],
    extended=True
)
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
