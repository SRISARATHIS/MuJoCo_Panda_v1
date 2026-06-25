from pathlib import Path

import mujoco
import mujoco.viewer


MODEL_PATH = Path(__file__).parent / "scene" / "scene_color.xml"


def main():
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)

    print(f"Loaded model: {MODEL_PATH}")
    print(f"Number of joints: {model.njnt}")
    print(f"Number of actuators: {model.nu}")

    mujoco.viewer.launch(model, data)


if __name__ == "__main__":
    main()