# MuJoCo Panda Teleoperation Station

This project implements a MuJoCo simulation scene for a Franka Emika Panda robot placed inside a custom teleoperation/data-collection workstation. The scene was created as part of a MuJoCo simulation assignment and includes a custom workstation layout, a Panda robot model, a basic Gymnasium-style environment wrapper, and viewer scripts for testing and interaction.

## Project Overview

The goal of this project is to build a realistic MuJoCo scene that represents a robotic data-collection setup. The scene contains:

* Franka Emika Panda robot arm
* Custom teleoperation workstation modeled in Blender
* Tables and robot mounting area
* Monitor, keyboard/control interface, and SpaceMouse
* Camera stands
* Backdrop/blinds
* Task objects and workstation elements
* Colored MuJoCo materials for better visual separation

The Panda model is integrated from the MuJoCo Menagerie Franka Emika Panda assets. The workstation objects were created and exported from Blender as separate OBJ mesh groups so that different parts can have different MuJoCo materials and colors.

## Repository Structure

```text
MuJoCo_Panda_v1/
├── env.py
├── run_viewer.py
├── live_env_viewer.py
├── requirements.txt
├── README.md
├── licenses/
│   └── README.md
├── scene/
│   ├── scene.xml
│   ├── scene_color.xml
│   └── assets/
│       ├── franka_emika_panda/
│       │   ├── panda.xml
│       │   └── robot mesh assets
│       └── meshes/
│           ├── tables.obj
│           ├── robo_mount.obj
│           ├── pc_spacemouse.obj
│           ├── blinds.obj
│           ├── camera_stands.obj
│           └── elements.obj
└── videos/
```

## Installation

These instructions assume Python is installed.

Clone the repository:

```bash
git clone https://github.com/SRISARATHIS/MuJoCo_Panda_v1.git
cd MuJoCo_Panda_v1
```

Create and activate a virtual environment:

```bash
python -m venv .venv
source .venv/bin/activate
```

Install dependencies:

```bash
pip install --upgrade pip
pip install -r requirements.txt
```

The required Python packages are:

```text
mujoco
gymnasium
numpy
```

## Running the MuJoCo Viewer

To open the main MuJoCo scene:

```bash
python run_viewer.py
```

This loads the MuJoCo XML scene and opens the standard MuJoCo viewer.

If using the colored scene version, make sure `run_viewer.py` or `live_env_viewer.py` points to:

```python
scene/scene_color.xml
```

## Running the Gymnasium Environment

The file `env.py` defines a minimal Gymnasium-style environment called `PandaTeleopStationEnv`.

Run:

```bash
python env.py
```

This tests:

* loading the MuJoCo model
* resetting the environment
* creating observations
* sampling actions
* stepping the simulation
* computing a simple reward based on end-effector distance to a target

The observation contains:

```text
qpos + qvel + end-effector position + target position
```

The action space uses the Panda actuator controls.

## Running the Live Viewer

The file `live_env_viewer.py` opens a MuJoCo viewer and prints robot state values in the terminal. This is the combination of MuJoCo viewer and Gymnasium style environment class. This gives clear and observble data for the user.

Run:

```bash
python live_env_viewer.py
```

Supported terminal commands:

```text
reset  -> return robot to home pose
q      -> quit viewer
```

The terminal prints readable robot state information including:

* joint positions
* joint velocities
* actuator controls
* gripper values

## Scene Design

The workstation was modeled in Blender and exported as separate OBJ mesh groups. This was done because a single OBJ export made the whole scene appear as one color in MuJoCo. By splitting the workstation into separate mesh files, MuJoCo can assign different materials to each part.

The scene currently uses separate meshes for:

```text
tables.obj
robo_mount.obj
pc_spacemouse.obj
blinds.obj
camera_stands.obj
elements.obj
```

Different MuJoCo materials are assigned to these meshes in `scene_color.xml`, for example:

* blinds: yellow
* camera stands: dark grey
* task elements: brown
* robot mount: dark/brown material
* background: blue skybox/blue haze

## Panda Robot Placement

The Panda robot is included from:

```xml
<include file="assets/franka_emika_panda/panda.xml"/>
```

The robot base position is controlled inside:

```text
scene/assets/franka_emika_panda/panda.xml
```

Specifically, the `link0` body position controls the Panda base placement:

```xml
<body name="link0" childclass="panda" pos="X Y Z" euler="0 0 1.5708">
```

The `Z` value controls height. Lowering `Z` moves the robot down; increasing `Z` moves the robot up.

## Gymnasium Environment Details

The Gymnasium-style environment implements:

```python
reset()
step(action)
```

The reward is based on the distance between the Panda end-effector and a fixed target position:

```text
reward = -distance_to_target
```

A success condition is triggered when the end-effector is close enough to the target.

This is a minimal environment intended to demonstrate integration between MuJoCo and Gymnasium. It is not a trained policy or full manipulation controller.

## Asset Sources

### Franka Emika Panda

The Panda robot model is based on the MuJoCo Menagerie Franka Emika Panda model from Google DeepMind.

### Workstation Meshes

The teleoperation workstation meshes were created and arranged in Blender for this project. The mesh groups were exported as OBJ files and loaded into MuJoCo.

## Known Limitations

This project is a simulation scene and basic environment wrapper, not a full manipulation benchmark. Current limitations include:

* The workstation meshes are mainly visual and do not all have detailed collision geometry.
* The Gymnasium environment uses a simple distance-based reward.
* Random actions do not solve the task; they are only used to test stepping.
* The Panda is manually positioned in the scene.
* The scene colors are assigned through MuJoCo materials rather than Blender materials.

Main commands:

```bash
python run_viewer.py
python env.py
python live_env_viewer.py
```
