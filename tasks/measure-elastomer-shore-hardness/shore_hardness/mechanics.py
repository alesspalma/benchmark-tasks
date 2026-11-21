"""Unilateral viscoelastic contact between the steel indenter and rubber coupon."""

import numpy as np
from yam_arena.urdf import quat_xyzw_to_matrix

from . import config as C


def frame(transform):
    return transform[:3], quat_xyzw_to_matrix(transform[3:])


def to_numpy(value):
    if hasattr(value, "torch"):
        value = value.torch
    if hasattr(value, "detach"):
        value = value.detach().cpu()
    return value.numpy()[0]


class CouponContact:
    """Use the current native poses at every physics substep."""

    def __init__(self, scene, torch):
        self.gauge = scene["durometer"]
        self.coupon = scene["coupon"]
        self.pin_id = self.gauge.body_names.index("Indenter")
        shape = (1, self.gauge.num_bodies, 3)
        tensor_options = {"device": self.gauge.device, "dtype": torch.float32}
        self.tool_force = torch.zeros(shape, **tensor_options)
        self.tool_moment = torch.zeros_like(self.tool_force)
        self.sample_force = torch.zeros((1, 1, 3), **tensor_options)
        self.sample_moment = torch.zeros_like(self.sample_force)
        self.tensor = lambda value: torch.as_tensor(value, **tensor_options)
        self.load = 0.
        previous_write = scene.write_data_to_sim

        def write():
            self.apply()
            previous_write()

        scene.write_data_to_sim = write

    def apply(self):
        tool_poses = to_numpy(self.gauge.root_view.get_link_transforms())
        pin, pin_rotation = frame(tool_poses[self.pin_id])
        coupon, sample_rotation = frame(to_numpy(self.coupon.root_view.get_transforms()))
        tip = pin - C.STROKE * pin_rotation[:, 2]
        local_tip = sample_rotation.T @ (tip - coupon)
        penetration = C.COUPON_SIZE[2] / 2 - local_tip[2]
        normal = sample_rotation[:, 2]
        within_face = np.all(np.abs(local_tip[:2]) <= C.COUPON_SIZE[:2] / 2)
        self.load = 0.
        if within_face and 0. < penetration < .005:
            pin_speed = to_numpy(self.gauge.root_view.get_link_velocities())[self.pin_id, :3]
            sample_velocity = to_numpy(self.coupon.root_view.get_velocities())
            sample_speed = sample_velocity[:3] + np.cross(sample_velocity[3:], tip - coupon)
            speed_into_sample = float((sample_speed - pin_speed) @ normal)
            self.load = float(np.clip(C.RUBBER_RATE * penetration + C.RUBBER_DAMPING * speed_into_sample, 0., 14.))
        force = self.load * normal
        self.tool_force[0, self.pin_id] = self.tensor(force)
        self.sample_force[0, 0] = self.tensor(-force)
        self.sample_moment[0, 0] = self.tensor(np.cross(tip - coupon, -force))
        for obj, f, tau in (
            (self.gauge, self.tool_force, self.tool_moment),
            (self.coupon, self.sample_force, self.sample_moment),
        ):
            wrench = obj.permanent_wrench_composer
            wrench.reset()
            wrench.add_forces_and_torques(f, tau, is_global=True)
