from pathlib import Path

import gymnasium as gym
import mujoco
import numpy as np
from gymnasium import spaces


class PandaTeleopStationEnv(gym.Env):
    """
    Gymnasium environment for the MuJoCo Franka Panda workstation scene.

    This environment exposes:
    - reset(): reset MuJoCo simulation
    - step(action): apply Panda actuator controls and advance simulation

    Observation:
        qpos + qvel + end-effector position + target position

    Action:
        MuJoCo actuator control vector, usually 8 values for this Panda model.
    """

    metadata = {
        "render_modes": [],
        "render_fps": 60,
    }

    def __init__(self, model_path=None, frame_skip=10, max_steps=200):
        super().__init__()

        if model_path is None:
            model_path = Path(__file__).parent / "scene" / "scene.xml"

        self.model_path = Path(model_path)
        self.model = mujoco.MjModel.from_xml_path(str(self.model_path))
        self.data = mujoco.MjData(self.model)

        self.frame_skip = frame_skip
        self.max_steps = max_steps
        self.current_step = 0

        # Simple fixed target above the task area.
        # You can tune this later based on your table/tray position.
        self.target_pos = np.array([0.45, 0.0, 0.75], dtype=np.float64)

        self.ee_body_id = self._find_body_id(["hand", "panda_hand", "link7"])

        if self.model.nu > 0:
            ctrl_range = self.model.actuator_ctrlrange.copy()
            low = ctrl_range[:, 0]
            high = ctrl_range[:, 1]

            low = np.where(np.isfinite(low), low, -1.0)
            high = np.where(np.isfinite(high), high, 1.0)

            self.action_space = spaces.Box(
                low=low.astype(np.float32),
                high=high.astype(np.float32),
                dtype=np.float32,
            )
        else:
            self.action_space = spaces.Box(
                low=np.array([], dtype=np.float32),
                high=np.array([], dtype=np.float32),
                dtype=np.float32,
            )

        obs_dim = self.model.nq + self.model.nv + 3 + 3

        self.observation_space = spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(obs_dim,),
            dtype=np.float64,
        )

    def _find_body_id(self, candidate_names):
        for name in candidate_names:
            body_id = mujoco.mj_name2id(
                self.model,
                mujoco.mjtObj.mjOBJ_BODY,
                name,
            )
            if body_id != -1:
                return body_id

        print("Warning: expected Panda end-effector body not found. Using last body.")
        return self.model.nbody - 1

    def _get_ee_pos(self):
        return self.data.xpos[self.ee_body_id].copy()

    def _get_obs(self):
        ee_pos = self._get_ee_pos()

        return np.concatenate(
            [
                self.data.qpos.copy(),
                self.data.qvel.copy(),
                ee_pos,
                self.target_pos.copy(),
            ]
        )

    def _compute_reward(self):
        ee_pos = self._get_ee_pos()
        distance = np.linalg.norm(ee_pos - self.target_pos)
        reward = -distance
        return reward, distance

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)

        self.current_step = 0

        mujoco.mj_resetData(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)

        obs = self._get_obs()
        reward, distance = self._compute_reward()

        info = {
            "model_path": str(self.model_path),
            "nq": self.model.nq,
            "nv": self.model.nv,
            "nu": self.model.nu,
            "end_effector_pos": self._get_ee_pos(),
            "target_pos": self.target_pos.copy(),
            "distance_to_target": distance,
            "initial_reward": reward,
        }

        return obs, info

    def step(self, action):
        self.current_step += 1

        action = np.asarray(action, dtype=np.float64)

        if self.model.nu > 0:
            action = np.clip(
                action,
                self.action_space.low,
                self.action_space.high,
            )
            self.data.ctrl[:] = action

        for _ in range(self.frame_skip):
            mujoco.mj_step(self.model, self.data)

        obs = self._get_obs()
        reward, distance = self._compute_reward()

        terminated = distance < 0.05
        truncated = self.current_step >= self.max_steps

        info = {
            "step": self.current_step,
            "end_effector_pos": self._get_ee_pos(),
            "target_pos": self.target_pos.copy(),
            "distance_to_target": distance,
            "success": terminated,
        }

        return obs, reward, terminated, truncated, info

    def close(self):
        pass


if __name__ == "__main__":
    env = PandaTeleopStationEnv()
    obs, info = env.reset()

    print("Environment loaded successfully.")
    print(f"Observation shape: {obs.shape}")
    print(f"Observation space: {env.observation_space}")
    print(f"Action space: {env.action_space}")
    print(f"Initial distance to target: {info['distance_to_target']:.4f}")
    print(f"Initial end-effector position: {info['end_effector_pos']}")
    print(f"Target position: {info['target_pos']}")
    print(f"Model info: nq={info['nq']}, nv={info['nv']}, nu={info['nu']}")

    for i in range(10):
        action = env.action_space.sample()
        obs, reward, terminated, truncated, info = env.step(action)

        print(
            f"Step {i + 1}: "
            f"reward={reward:.4f}, "
            f"distance={info['distance_to_target']:.4f}, "
            f"success={info['success']}"
        )

        if terminated or truncated:
            break

    env.close()