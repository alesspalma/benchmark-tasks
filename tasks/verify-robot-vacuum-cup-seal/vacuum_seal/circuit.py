"""A compressible, checkable test circuit acting on the native test piece."""

from math import exp, pi

import numpy as np
from yam_arena.urdf import quat_xyzw_to_matrix

from . import config as C


class Circuit:
    def __init__(self):
        self.reset()
        angles = np.arange(12) * pi / 6
        self.rim = np.column_stack(
            (C.LIP_R * np.cos(angles), C.LIP_R * np.sin(angles), np.zeros(12))
        )

    def reset(self):
        self.pressure = 0.0
        self.sealed = False
        self.supply = False
        self.vent = False
        self.compression = 0.0
        self.max_gap = 0.0
        self.rim_load = 0.0
        self.rim_contact = False

    def advance(self, joints, plate_pose, lip_pose, dt, *, rim_load, rim_contact):
        """Sample lip geometry and return the pneumatic load on the test piece."""
        bellows = float(joints["BellowsTravel"])
        spool = float(joints["SupplyTravel"])
        self.supply = spool >= 0.018 if not self.supply else spool > 0.010
        self.vent = float(joints["VentTravel"]) <= -0.0045
        self.compression = max(0.0, bellows)
        self.rim_load = rim_load
        self.rim_contact = rim_contact
        axes = quat_xyzw_to_matrix(plate_pose[3:])
        lip_axes = quat_xyzw_to_matrix(lip_pose[3:])
        sectors = self.rim @ lip_axes.T + lip_pose[:3]
        local = (sectors - plate_pose[:3]) @ axes
        offsets = local[:, 2] - C.PLATE_SIZE[2] / 2
        self.max_gap = float(np.max(offsets))
        covered = bool(
            bellows >= C.MIN_BELLOWS_COMPRESSION
            and axes[:, 2] @ lip_axes[:, 2] > 0.998
            and np.all(np.abs(local[:, :2]) <= C.PLATE_SIZE[:2] / 2 - 0.002)
            and np.min(offsets) > -0.0024
        )
        self.sealed = bool(
            covered and rim_contact and rim_load >= C.MIN_RIM_LOAD_N
            and self.max_gap <= C.LIP_LOCAL_COMPLIANCE
        )
        # A local peel breaks the scored seal and starts leakage. The
        # remaining contact restricts flow and preserves a transient load.
        if self.sealed:
            force_fraction = 1.0
        elif covered and rim_contact:
            force_fraction = np.clip(2.0 - self.max_gap / C.LIP_LOCAL_COMPLIANCE, 0.0, 1.0)
        else:
            force_fraction = 0.0
        if self.vent:
            target, time_constant = 0.0, 0.045
        elif not self.sealed:
            target = 1.5 if self.supply else 0.0
            time_constant = 0.055 + 0.45 * force_fraction
        elif self.supply:
            target, time_constant = C.LOAD_PRESSURE_KPA, 0.12
        else:
            target, time_constant = 0.0, 40.0
        self.pressure = target + (self.pressure - target) * exp(-dt / time_constant)
        if force_fraction == 0.0:
            return np.zeros(3), np.zeros(3)
        # Atmospheric pressure carries the covered bore area. The force acts
        # through its centre, leaving lateral friction and moment restraint to
        # native contact with the compressed lip.
        force = axes[:, 2] * (1000.0 * self.pressure * pi * C.INNER_RADIUS**2 * force_fraction)
        application = sectors.mean(axis=0)
        return force, np.cross(application - plate_pose[:3], force)
