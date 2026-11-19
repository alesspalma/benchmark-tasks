"""Inspect the released optic, retaining ring and unobstructed aperture."""

import numpy as np

from . import config as C


def local(point, reference, frame):
    return frame.T @ (np.asarray(point) - reference)


class Scorer:
    def __init__(self):
        self.reset()

    def reset(self):
        self.face_contact = False
        self.released_since = None
        self.release_angle = None

    def evaluate(self, state):
        eye, basis = np.asarray(state["lens_pos"]), np.asarray(state["lens_frame"])
        tube, frame = np.asarray(state["tube_pos"]), np.asarray(state["tube_frame"])
        lens_offset = local(eye, tube, frame)
        lens_tilt = float(np.dot(basis[:, 2], frame[:, 2]))
        # Check the two pins and cross member over the clear coated face. The
        # annular rim under the ring's load face may be contacted safely.
        driver = np.asarray(state["tool_pos"])
        tool_frame = np.asarray(state["tool_frame"])
        tips = [
            driver + tool_frame @ np.array((x, y, z))
            for x, y, z in (
                (C.TOOL_TEETH_RADIUS-.00050, -.0010, 0.),
                (C.TOOL_TEETH_RADIUS-.00050, .0010, 0.),
                (-C.TOOL_TEETH_RADIUS+.00050, -.0010, 0.),
                (-C.TOOL_TEETH_RADIUS+.00050, .0010, 0.),
                (.011, 0., .017), (-.011, 0., .017), (0., 0., .023),
            )
        ]
        for point in tips:
            x, y, z = local(point, eye, basis)
            if x*x + y*y < C.CLEAR_RADIUS**2 and -.0022 < z < .00218:
                self.face_contact = True

        for coords in state["fingertips"].values():
            for point in coords:
                x, y, z = local(point, eye, basis)
                if x*x+y*y < (C.CLEAR_RADIUS - .0007)**2 and -.001 < z < .00205:
                    self.face_contact = True

        alignment = float(np.linalg.norm(lens_offset[:2]))
        seat_error = abs(lens_offset[2] - C.LENS_CENTER)
        flush = bool(alignment < .00065 and seat_error < .00018 and lens_tilt > .999)
        ring_plane = C.RING_BOTTOM + state["lead"]
        rim = lens_offset[2] + C.LENS_THICKNESS/2
        ring_gap = ring_plane - rim
        angular_advance = -state["angle"]
        pitch_gap = C.RING_CLEARANCE - C.PITCH_PER_RADIAN * angular_advance
        preload = float(min(C.THREAD_FORCE_LIMIT, max(0., 75000. * (ring_gap - pitch_gap))))
        # Measured clearance and straight-through axis through the retainer,
        # optic seat and the 18 mm inspection target on the light pad.
        z_axis = frame[:, 2]
        axis_hit = eye - z_axis * (eye[2]-C.FIXTURE_TOP) / max(.001, z_axis[2])
        centered_on_light = float(np.linalg.norm(axis_hit[:2] - C.TUBE[:2])) < .009
        open_axis = bool(centered_on_light and alignment + .019 < C.CLEAR_RADIUS)
        resting = (abs(tube[2]-C.FIXTURE_TOP) < .001 and z_axis[2] > .997
                   and state["tube_speed"] < .015 and state["lens_speed"] < .013)
        clear = all(np.linalg.norm(np.asarray(p) - (tube + .03 * z_axis)) > .067
                    for p in state["hands"].values())
        tool_clear = all(np.linalg.norm(point - eye) > .055 for point in tips)
        pressed = bool(preload >= .9 and -.00016 < ring_gap < .00015)
        advanced = bool(angular_advance > .46 and ring_gap < C.RING_CLEARANCE - .000025)
        if clear and tool_clear and resting and flush and open_axis and advanced:
            if self.released_since is None or abs(state["angle"] - self.release_angle) > .0524:
                self.released_since = state["time_s"]
                self.release_angle = state["angle"]
        else:
            self.released_since = None
        released = self.released_since is not None and state["time_s"] - self.released_since >= .70
        score = (1. if released and pressed else .5 if released else 0.) if not self.face_contact else 0.
        return {
            "score": score,
            "success": score == 1.,
            "flush_seat": flush,
            "clear_light_aperture": open_axis,
            "contact_with_clear_face": self.face_contact,
            "tool_clear": tool_clear,
            "hands_clear": bool(clear),
            "released": released,
            "angle_deg": float(np.degrees(state["angle"])),
            "lead_um": float(1e6 * state["lead"]),
            "optic_center_error_um": 1e6 * alignment,
            "optic_seat_error_um": 1e6 * seat_error,
            "retaining_gap_um": 1e6 * ring_gap,
            "rim_preload_n": preload,
        }
