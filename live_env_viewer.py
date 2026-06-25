from pathlib import Path
import select
import sys
import time

import mujoco
import mujoco.viewer
import numpy as np


MODEL_PATH = Path(__file__).parent / "scene" / "scene.xml"

HOME_QPOS = np.array([
    0.0,      # joint 1
    -0.785,   # joint 2
    0.0,      # joint 3
    -2.356,   # joint 4
    0.0,      # joint 5
    1.571,    # joint 6
    0.785,    # joint 7
    0.04,     # left finger
    0.04,     # right finger
], dtype=np.float64)


def reset_to_home(model, data):

    mujoco.mj_resetData(model, data)
    n = min(model.nq, len(HOME_QPOS))
    data.qpos[:n] = HOME_QPOS[:n]
    data.qvel[:] = 0.0

    if model.nu >= 7:
        data.ctrl[:7] = HOME_QPOS[:7]

    if model.nu >= 8:
        data.ctrl[7] = 0.0

    mujoco.mj_forward(model, data)


def get_observation(model, data):

    return np.concatenate([
        data.qpos.copy(),
        data.qvel.copy(),
        data.ctrl.copy(),
    ])


def print_state(data):

    print("\n--- Robot state changed ---")
    print(f"qpos: {np.round(data.qpos, 4)}")
    print(f"qvel: {np.round(data.qvel, 4)}")
    print(f"ctrl: {np.round(data.ctrl, 4)}")
    print("Type 'reset' + Enter to return to home pose, or 'q' + Enter to quit.")


def terminal_command_available():

    readable, _, _ = select.select([sys.stdin], [], [], 0)
    return bool(readable)


def main():
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)

    reset_to_home(model, data)

    print(f"Loaded model: {MODEL_PATH}")
    print(f"Number of joints: {model.njnt}")
    print(f"Number of actuators: {model.nu}")
    print("Starting from home pose.")
    print("Move joints/actuators in the MuJoCo viewer.")
    print("This terminal prints only when qpos/qvel/ctrl changes.")
    print("Commands: reset, q")

    previous_obs = get_observation(model, data)
    print_state(data)

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            # Step simulation.
            mujoco.mj_step(model, data)

            # Sync viewer with simulation.
            viewer.sync()

            # Check terminal commands.
            if terminal_command_available():
                command = sys.stdin.readline().strip().lower()

                if command == "reset":
                    reset_to_home(model, data)
                    previous_obs = get_observation(model, data)
                    print("\nReset to home pose.")
                    print_state(data)

                elif command in {"q", "quit", "exit"}:
                    print("Exiting live viewer.")
                    break

            current_obs = get_observation(model, data)

            if not np.allclose(current_obs, previous_obs, atol=1e-4):
                print_state(data)
                previous_obs = current_obs.copy()

            time.sleep(0.01)


if __name__ == "__main__":
    main()