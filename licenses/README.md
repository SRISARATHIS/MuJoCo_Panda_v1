# Licenses and Asset Attribution

This project uses a combination of self-created assets and open-source robotics assets.

## Franka Emika Panda Robot Model

The Franka Emika Panda robot model used in this project is based on the Franka Emika Panda model from Google DeepMind's MuJoCo Menagerie.

Source:

```text
https://github.com/google-deepmind/mujoco_menagerie
```

The Panda model files used in this project are located in:

```text
scene/assets/franka_emika_panda/
```

Attribution:

```text
Franka Emika Panda model from Google DeepMind MuJoCo Menagerie.
```

The original MuJoCo Menagerie license should be respected when using or redistributing these assets. A copy of or reference to the original license should be kept with the project.

## MuJoCo

This project uses MuJoCo through the Python `mujoco` package.

Source:

```text
https://github.com/google-deepmind/mujoco
```

MuJoCo is developed and maintained by Google DeepMind.

## Gymnasium

This project uses Gymnasium for the minimal environment interface.

Source:

```text
https://gymnasium.farama.org/
```

The custom environment class in this project follows the Gymnasium-style API by implementing:

```text
reset()
step(action)
```

## Custom Blender Workstation Assets

The teleoperation/data-collection workstation assets were created by the project author in Blender for this project.

The custom workstation mesh files are located in:

```text
scene/assets/meshes/
```

The main custom OBJ assets include:

```text
tables.obj
robo_mount.obj
pc_spacemouse.obj
blinds.obj
camera_stands.obj
elements.obj
```

These assets were created for this MuJoCo simulation task.

## Project Code

The Python code in this repository was written for this project.

Main project files include:

```text
run_viewer.py
env.py
live_env_viewer.py
```

The code is intended for demonstration of a MuJoCo simulation scene and minimal Gymnasium-style environment.

## Notes

No paid commercial assets were used in this project.

If additional third-party assets are added in the future, their original source, author, and license should be added to this file.
