"""Collect clear matched image pairs and validate the stereo fit."""

from __future__ import annotations

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from . import config as C

STRIDE = 4
OBJECT = np.zeros((C.GRID[0]*C.GRID[1], 3), dtype=np.float32)
OBJECT[:, :2] = np.mgrid[:C.GRID[0], :C.GRID[1]].T.reshape(-1, 2)*C.SQUARE
NOMINAL_FOCAL = C.FOCAL / C.APERTURE * C.IMAGE_SIZE[0]
NOMINAL_K = np.array(
    [[NOMINAL_FOCAL, 0., C.IMAGE_SIZE[0]/2],
     [0., NOMINAL_FOCAL, C.IMAGE_SIZE[1]/2],
     [0., 0., 1.]]
)


def ideal_images(pose, hands):
    """Fast pinhole rasterization for physical CPU-only development runs."""
    pos, quat = np.asarray(pose[:3]), np.asarray(pose[3:])
    rot = Rotation.from_quat(quat).as_matrix()
    cols, rows = C.GRID[0]+1, C.GRID[1]+1
    local = np.array([
        [C.PATTERN_X, (col-cols/2)*C.SQUARE, (rows/2-row)*C.SQUARE]
        for row in range(rows+1) for col in range(cols+1)
    ])
    world = local@rot.T + pos
    images = []
    for camera in C.CAMERAS:
        cam = np.asarray(camera)

        def project(points):
            delta = np.asarray(points)-cam
            depth = -delta[:, 0]
            uv = NOMINAL_FOCAL * np.column_stack((delta[:, 1], -delta[:, 2])) / np.maximum(depth[:, None], .02)
            return uv + NOMINAL_K[:2, 2], depth

        uv, depth = project(world)
        uv = np.rint(uv * STRIDE).astype(np.int32)
        gray = np.full((C.IMAGE_SIZE[1]*STRIDE, C.IMAGE_SIZE[0]*STRIDE), 116, dtype=np.uint8)
        if depth.min() > .05 and rot[0, 0] > 0.3:
            outer = np.array([
                [C.PATTERN_X, y*C.SIZE[1]/2, z*C.SIZE[2]/2]
                for y, z in ((-1, 1), (1, 1), (1, -1), (-1, -1))
            ])
            board_uv, _ = project(outer@rot.T+pos)
            cv2.fillConvexPoly(gray, np.rint(board_uv*STRIDE).astype(np.int32), 235)
            for row in range(rows):
                for col in range(cols):
                    i = row*(cols+1)+col
                    idx = [i, i+1, i+cols+2, i+cols+1]
                    cv2.fillConvexPoly(gray, uv[idx], 33 if (row+col) % 2 == 0 else 232)
            for hand in hands.values():
                for point, radius in zip(hand["points"], (.006, .007, .015, .008, .008)):
                    at, d = project([point])
                    if .05 < d[0] < np.median(depth)-.004:
                        center = tuple(np.rint(at[0]*STRIDE).astype(int))
                        cv2.circle(gray, center, max(1, round(NOMINAL_FOCAL*radius*STRIDE/d[0])), 83, -1)
        images.append(cv2.resize(gray, C.IMAGE_SIZE, interpolation=cv2.INTER_AREA))
    return images


def corners_in(image):
    gray = image if image.ndim == 2 else cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    flags = cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_ACCURACY
    found, corners = cv2.findChessboardCornersSB(gray, C.GRID, flags)
    if not found:
        return None
    points = corners[:, 0]
    # The two parallel cameras see the same side of the same regular grid.
    # Both detections start at the upper-left grid corner.
    if points[0].sum() > points[-1].sum():
        points = points[::-1]
    span = np.ptp(points, axis=0)
    if np.any(points.min(axis=0) < 12) or np.any(points.max(axis=0) > np.array(C.IMAGE_SIZE)-12):
        return None
    if span[0] < 73 or span[1] < 47:
        return None
    return np.ascontiguousarray(points, dtype=np.float32)


def view_parameters(points, intrinsic):
    valid, rotation, _ = cv2.solvePnP(OBJECT, points, intrinsic, None)
    if not valid:
        return None
    normal = cv2.Rodrigues(rotation)[0][:, 2]
    attitude = np.rad2deg(np.arctan2(normal[:2], abs(normal[2])))
    center = points.mean(axis=0)
    scale = np.linalg.norm(points[C.GRID[0]-1]-points[0])
    return np.r_[center, scale, attitude]


def collection_coverage(parameters):
    if len(parameters) < 2:
        return np.zeros(5)
    return np.ptp(parameters, axis=0)


def stereo_fit(pairs, matrices):
    """Fit camera extrinsics and measure rectification on a withheld image pair."""
    if len(pairs) < C.ACCEPTED_PAIRS:
        return None
    fit_pairs = pairs[:-1]
    left, right = ([p[i] for p in fit_pairs] for i in range(2))
    shape = [OBJECT]*len(fit_pairs)
    try:
        outcome = cv2.stereoCalibrate(
            shape, left, right, *[
                item for k in matrices for item in (k.copy(), np.zeros(5))
            ], C.IMAGE_SIZE, flags=cv2.CALIB_FIX_INTRINSIC,
            criteria=(cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, 35, 1e-6),
        )
        zero = np.zeros(5)
        rot, offset = outcome[5:7]
        rect_left, rect_right, map_left, map_right, *_ = cv2.stereoRectify(
            matrices[0], zero, matrices[1], zero, C.IMAGE_SIZE, rot, offset
        )
        held = [
            cv2.undistortPoints(pairs[-1][i], matrices[i], zero, R=r, P=m)[:, 0]
            for i, (r, m) in enumerate(zip((rect_left, rect_right), (map_left, map_right)))
        ]
    except cv2.error:
        return None
    return dict(
        stereo_rms_px=float(outcome[0]),
        recovered_baseline_mm=float(1000*np.linalg.norm(offset)),
        held_out_epipolar_rms_px=float(np.sqrt(np.mean(np.square(held[0][:, 1]-held[1][:, 1])))),
        rotation_left_to_right=rot.tolist(),
        translation_left_to_right_mm=(1000*offset).ravel().tolist(),
    )


class Scorer:
    def __init__(self):
        self.reset()

    def reset(self):
        self.previous = None
        self.still_time = 0.
        self.tried_pose = False
        self.views = []
        self.pairs = []
        self.images = []
        self.candidate_misses = 0
        self.fit = None
        self.return_time = 0.
        self.matrices = [NOMINAL_K.copy(), NOMINAL_K.copy()]
        self.source = "analytical development cameras"

    def evaluate(self, state, read_images, elapsed):
        pose = np.asarray(state["objects"]["checkerboard"]["pose"])
        rot = Rotation.from_quat(pose[3:]).as_matrix()
        extent = np.abs(rot[2]) @ (np.asarray(C.SIZE)/2)
        clear_of_stand = pose[2]-extent > C.TABLE+.040
        if self.previous is not None:
            angle_delta = 2*np.arccos(np.clip(abs(pose[3:]@self.previous[3:]), 0., 1.))
            still = np.linalg.norm(pose[:3]-self.previous[:3]) < .0007 and angle_delta < .010
        else:
            still = False
        self.previous = pose
        self.still_time = self.still_time+elapsed if still and clear_of_stand else 0.
        if self.still_time == 0.:
            self.tried_pose = False
        if self.still_time >= .30 and not self.tried_pose and len(self.pairs) < C.ACCEPTED_PAIRS:
            self.tried_pose = True
            frames, matrices, self.source = read_images(state)
            self.matrices = matrices
            points = tuple(corners_in(image) for image in frames)
            if all(p is not None for p in points):
                values = view_parameters(points[0], matrices[0])
                if values is not None:
                    # Change in either image position, projected size, or skew must
                    # be substantial compared with every already accepted image.
                    distance = np.abs(np.asarray(self.views)-values)/[28., 25., 18., 9., 9.] if self.views else []
                    duplicate = any(np.max(d) < 1. for d in distance)
                    if not duplicate:
                        self.views.append(values)
                        self.pairs.append(points)
                        self.images.append(tuple(frame.copy() for frame in frames))
                        self.fit = stereo_fit(self.pairs, matrices)
            else:
                self.candidate_misses += 1

        coverage = collection_coverage(self.views)
        qualified = bool(
            len(self.pairs) == C.ACCEPTED_PAIRS and self.fit is not None
            and self.fit["stereo_rms_px"] <= .80 and self.fit["held_out_epipolar_rms_px"] <= 1.10
            and 84. <= self.fit["recovered_baseline_mm"] <= 132.
            and np.all(coverage >= (44., 33., 19., 11., 14.))
        )
        seated = (
            abs(pose[0]-C.BOARD[0]) <= .018 and abs(pose[1]-C.BOARD[1]) <= .019
            and abs(pose[2]-C.BOARD[2]) <= .014 and rot[2, 2] >= .96 and rot[0, 0] >= .96
        )
        hands_clear = all(
            np.any(np.abs((np.asarray(point)-pose[:3])@rot) > (.026, .176, .116))
            for hand in state["hands"].values() for point in hand["points"][:2]
        )
        self.return_time = self.return_time+elapsed if seated and hands_clear else 0.
        complete = qualified and self.return_time >= .35
        score = 1. if complete else .82 if qualified else .09*len(self.pairs)
        return dict(
            score=round(score, 4), success=bool(complete), pairs=len(self.pairs),
            center_span_x_px=float(coverage[0]), center_span_y_px=float(coverage[1]),
            width_span_px=float(coverage[2]), tilt_span_x_deg=float(coverage[3]),
            tilt_span_y_deg=float(coverage[4]), stereo_fit=self.fit,
            board_in_stand=bool(seated), hands_clear=bool(hands_clear),
            low_quality_pairs=self.candidate_misses, camera_source=self.source,
        )
