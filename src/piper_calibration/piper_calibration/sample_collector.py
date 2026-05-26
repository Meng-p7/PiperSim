import numpy as np
import random


DEFAULT_JOINT_LIMITS = np.array([
    [-2.618, 2.618],
    [0.0, 3.14],
    [-2.697, 0.0],
    [-1.832, 1.832],
    [-1.22, 1.22],
    [-3.14, 3.14],
])

DEFAULT_NOISE_SCALE = np.array([0.05, 0.08, 0.08, 0.10, 0.05, 0.10])


class SampleCollector:
    def __init__(
        self,
        num_poses: int = 15,
        seed_joints: list[float] | None = None,
        noise_per_joint: list[float] | None = None,
        joint_limits: np.ndarray | None = None,
    ):
        self.num_poses = num_poses
        self.seed = np.array(seed_joints or [0.0, 1.57, -1.3485, 0.0, 0.0, 0.0])
        self.noise_scale = np.array(noise_per_joint or DEFAULT_NOISE_SCALE)
        self.limits = joint_limits if joint_limits is not None else DEFAULT_JOINT_LIMITS

    def generate(self) -> list[np.ndarray]:
        rng = np.random.RandomState()
        rng.seed(random.randint(0, 2 ** 16))

        poses = []
        poses.append(self.seed.copy())

        max_attempts = self.num_poses * 200
        attempts = 0
        while len(poses) < self.num_poses and attempts < max_attempts:
            attempts += 1
            noise = rng.randn(6) * self.noise_scale
            q = np.clip(self.seed + noise, self.limits[:, 0], self.limits[:, 1])
            if self._is_far_enough(q, poses):
                poses.append(q)

        return poses

    @staticmethod
    def _is_far_enough(q: np.ndarray, existing: list[np.ndarray], min_dist: float = 0.1) -> bool:
        for e in existing:
            if np.linalg.norm(q - e) < min_dist:
                return False
        return True
