"""Score a cold, released gap and a native sliding paper contact."""

import numpy as np
from yam_arena.urdf import quat_xyzw_to_matrix

from . import config as C


def pose(points):
    return np.asarray(points[:3]), quat_xyzw_to_matrix(np.asarray(points[3:]))


def under_nozzle(state):
    paper_position, paper_rotation = pose(state["objects"]["paper"]["pose"])
    paper_local = paper_rotation.T @ (C.NOZZLE - paper_position)
    return (
        abs(paper_local[0]) < C.PAPER_SIZE[0]/2 - .001
        and abs(paper_local[1]) < C.PAPER_SIZE[1]/2 - .001
        and abs(paper_local[2]) < .00046
        and paper_rotation[2, 2] > .98
    )


def physical_gap(state):
    bed_position, bed_rotation = pose(state["objects"]["printer"]["links"]["BuildPlate"])
    normal = bed_rotation[:, 2]
    return float(np.dot(normal, C.NOZZLE-bed_position) / normal[2])


class Scorer:
    def reset(self):
        self.prior_paper = None
        self.initial_wheel = None
        self.prior_contact = False
        self.travel = 0.
        self.dwell = 0.
        self.contact = False

    def evaluate(self, state):
        gap = physical_gap(state)
        q = state["objects"]["printer"]["joints"]
        paper = state["objects"]["paper"]
        pos, rot = pose(paper["pose"])
        if self.initial_wheel is None:
            self.initial_wheel = q["Screw0"]
        adjusted = self.initial_wheel - q["Screw0"] > np.deg2rad(2.)
        load = C.NOZZLE_STIFFNESS * max(0., q["NozzleFlex"])
        gauge_ok = .000065 <= gap <= .000101
        intact = (
            np.all(np.abs(pos[:2] - C.BED_CENTER[:2]) < C.BED_SIDE/2 + .007)
            and abs(pos[2] - C.BED_CENTER[2]) < .003
        )
        under = under_nozzle(state)
        self.contact = bool(adjusted and gauge_ok and intact and under and .013 <= load <= .32)
        if self.contact and self.prior_contact:
            motion = np.linalg.norm(pos[:2] - self.prior_paper[:2])
            if .00005 < motion < .003:
                self.travel += motion
        self.prior_contact = self.contact
        self.prior_paper = pos
        checked = self.travel >= .012

        pad_points = [
            np.asarray(point)
            for hand in state["hands"].values()
            for point in hand["points"][:2]
        ]
        wheel_clear = all(np.linalg.norm(point - C.WHEEL) > .048 for point in pad_points)
        paper_clear = all(
            np.any(np.abs(rot.T @ (point - pos)) > (C.PAPER_SIZE/2 + [.020, .020, .022]))
            for point in pad_points
        )
        bed_clear = all(
            np.any(np.abs(point[:2] - C.BED_CENTER[:2]) > C.BED_SIDE/2 + .010)
            or abs(point[2] - C.BED_CENTER[2]) > .025
            for point in pad_points
        )
        clear = wheel_clear and paper_clear and bed_clear
        settled = np.linalg.norm(paper["velocity"][:3]) < .005
        staged = adjusted and gauge_ok and checked and clear and settled and intact
        self.dwell = self.dwell + C.CHECK_DT if staged else 0.
        success = self.dwell >= 1.
        score = 0.
        if adjusted and wheel_clear:
            if success:
                score = 1.
            elif gauge_ok:
                score = .75 if checked else .5
            elif .000101 < gap <= .000135:
                score = .25
        return dict(
            score=score, success=success, gap_mm=gap*1000,
            wheel_deg=float(np.rad2deg(q["Screw0"])), paper_travel_mm=self.travel*1000,
            paper_drag_contact=self.contact, nozzle_load_n=load, paper_on_bed=bool(intact),
            paper_under_nozzle=bool(under), clear=bool(clear), stable_s=self.dwell,
        )
