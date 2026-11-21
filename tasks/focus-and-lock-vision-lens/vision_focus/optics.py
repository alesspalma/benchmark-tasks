"""Image of the fixed transmissive line chart from the inspection sensor."""

from functools import lru_cache
from pathlib import Path

import numpy as np

from . import config as C

IMAGE_SHAPE = (112, 224)
LEVELS = 65
MAX_DISK_PIXELS = 32.0


def chart():
    image = np.full(IMAGE_SHAPE, 224, dtype=np.uint8)
    line_centers = (39, 53, 67, 81, 95, 109, 123, 137, 151, 165, 179, 193)
    for center in line_centers:
        image[31:87, center - 3 : center + 4] = 23
    image[13:16, 100:125] = 55
    image[7:22, 111:114] = 55
    return image


def disk_diameter(angle):
    """Geometrical circle of confusion measured in preview pixels."""
    f = C.FOCAL_LENGTH
    image_distance = f * C.TARGET_DISTANCE / (C.TARGET_DISTANCE - f)
    aperture = f / C.F_NUMBER
    sensor_error = (angle - C.FOCUS_ANGLE) * C.SENSOR_TRAVEL_PER_RADIAN
    magnification = image_distance / C.TARGET_DISTANCE
    image_m_per_pixel = magnification * C.CHART_SPAN / IMAGE_SHAPE[1]
    return abs(aperture * sensor_error / image_distance) / image_m_per_pixel


def blur_level(angle):
    diameter = min(MAX_DISK_PIXELS, disk_diameter(angle))
    return round((LEVELS - 1) * diameter / MAX_DISK_PIXELS)


@lru_cache(maxsize=LEVELS)
def preview(level):
    import cv2

    diameter = level * MAX_DISK_PIXELS / (LEVELS - 1)
    radius = diameter / 2.0
    bound = int(np.ceil(radius))
    y, x = np.ogrid[-bound : bound + 1, -bound : bound + 1]
    weights = np.clip(radius + 0.55 - np.hypot(x, y), 0, 1)
    weights /= weights.sum()
    image = cv2.filter2D(chart(), -1, weights, borderType=cv2.BORDER_REFLECT)
    return cv2.GaussianBlur(image, (5, 5), 0.65)


def contrast(angle):
    row = preview(blur_level(angle))[54:69].mean(axis=0)
    dark = np.mean(row[np.arange(53, 180, 14)])
    light = np.mean(row[np.arange(60, 180, 14)])
    return float(np.clip((light - dark) / (light + dark), 0, 1))


def write_display_images(directory):
    import cv2

    directory = Path(directory)
    for level in range(LEVELS):
        cv2.imwrite(str(directory / f"preview-{level:02}.png"), preview(level))
