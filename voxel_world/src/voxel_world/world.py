"""Small dense voxel world built from SDF collision boxes.

The first version intentionally supports box collision geometry only. It is
enough for the current excavation_long SDF and fails loudly on unsupported
geometry instead of silently producing a misleading map.
"""

from dataclasses import dataclass
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


@dataclass(frozen=True)
class BoxCollision:
    name: str
    size: np.ndarray
    rotation: np.ndarray
    translation: np.ndarray


def _pose(element):
    if element is None or not (element.text or '').strip():
        return np.eye(4)
    values = np.fromstring(element.text, sep=' ')
    if values.size != 6:
        raise ValueError('SDF pose must contain x y z roll pitch yaw')
    x, y, z, roll, pitch, yaw = values
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    transform = np.eye(4)
    transform[:3, :3] = rz @ ry @ rx
    transform[:3, 3] = [x, y, z]
    return transform


def load_sdf_boxes(path):
    """Load collision boxes from an SDF model or world file.

    SDF model, link, and collision poses are composed. ``relative_to`` frame
    attributes, included models, and non-box collision geometry are not yet
    supported; those cases raise an error so validation remains explicit.
    """
    root = ET.parse(str(path)).getroot()
    model = root.find('.//model')
    if model is None:
        raise ValueError('No <model> found in %s' % path)
    if model.get('relative_to') is not None:
        raise ValueError('SDF model pose relative_to is not supported')
    model_tf = _pose(model.find('pose'))
    boxes = []
    for link in model.findall('link'):
        if link.get('relative_to') is not None:
            raise ValueError('SDF link relative_to is not supported')
        link_tf = model_tf @ _pose(link.find('pose'))
        for collision in link.findall('collision'):
            if collision.get('relative_to') is not None:
                raise ValueError('SDF collision relative_to is not supported')
            geometry = collision.find('geometry')
            box = geometry.find('box') if geometry is not None else None
            if box is None:
                raise ValueError('Unsupported collision geometry in %s: %s' %
                                 (path, collision.get('name', '<unnamed>')))
            size = np.fromstring((box.findtext('size') or ''), sep=' ')
            if size.size != 3 or np.any(size <= 0):
                raise ValueError('Invalid box size in %s' % collision.get('name', '<unnamed>'))
            tf = link_tf @ _pose(collision.find('pose'))
            boxes.append(BoxCollision(
                collision.get('name', 'box'), size, tf[:3, :3].copy(), tf[:3, 3].copy()))
    if not boxes:
        raise ValueError('No box collision geometry found in %s' % path)
    return boxes


class VoxelWorld:
    """Axis-aligned bounded world; ``occupied[z, y, x]`` stores ground truth."""

    def __init__(self, origin, size, resolution):
        self.origin = np.asarray(origin, dtype=np.float64)
        self.size = np.asarray(size, dtype=np.float64)
        self.resolution = float(resolution)
        if self.origin.shape != (3,) or self.size.shape != (3,):
            raise ValueError('origin and size must each contain x, y, z')
        if self.resolution <= 0 or np.any(self.size <= 0):
            raise ValueError('resolution and map dimensions must be positive')
        self.shape_xyz = np.ceil(self.size / self.resolution).astype(np.int32)
        self.occupied = np.zeros(tuple(self.shape_xyz[::-1]), dtype=np.uint8)

    @classmethod
    def from_sdf(cls, path, origin, size, resolution):
        world = cls(origin, size, resolution)
        for box in load_sdf_boxes(path):
            world.add_box(box)
        return world

    def add_box(self, box):
        """Rasterize box voxels whose centers lie inside the SDF collision box."""
        half = box.size * 0.5
        corners = np.array([[sx, sy, sz]
                            for sx in (-half[0], half[0])
                            for sy in (-half[1], half[1])
                            for sz in (-half[2], half[2])])
        world_corners = corners @ box.rotation.T + box.translation
        low = np.floor((world_corners.min(axis=0) - self.origin) / self.resolution).astype(int)
        high = np.ceil((world_corners.max(axis=0) - self.origin) / self.resolution).astype(int)
        low = np.maximum(low, 0)
        high = np.minimum(high, self.shape_xyz)
        if np.any(high <= low):
            return

        xs = self.origin[0] + (np.arange(low[0], high[0]) + 0.5) * self.resolution
        ys = self.origin[1] + (np.arange(low[1], high[1]) + 0.5) * self.resolution
        inv_rotation = box.rotation.T
        for zi in range(low[2], high[2]):
            z = self.origin[2] + (zi + 0.5) * self.resolution
            yy, xx = np.meshgrid(ys, xs, indexing='ij')
            points = np.stack((xx - box.translation[0],
                               yy - box.translation[1],
                               np.full_like(xx, z - box.translation[2])), axis=-1)
            local = points @ inv_rotation.T
            inside = np.all(np.abs(local) <= half + 1e-10, axis=-1)
            self.occupied[zi, low[1]:high[1], low[0]:high[0]][inside] = 1

    def index(self, position):
        """Return (x,y,z) voxel index, or None outside map bounds."""
        idx = np.floor((np.asarray(position) - self.origin) / self.resolution).astype(int)
        if np.any(idx < 0) or np.any(idx >= self.shape_xyz):
            return None
        return idx

    def is_occupied(self, position):
        idx = self.index(position)
        return False if idx is None else bool(self.occupied[idx[2], idx[1], idx[0]])

    def raycast(self, origin, direction, max_range):
        """Return distance to first occupied voxel; ``None`` means no hit.

        Uses Amanatides-Woo voxel traversal. The map boundary is treated as
        unknown/free, matching the bounded map behavior in the Gazebo mapper.
        """
        origin = np.asarray(origin, dtype=np.float64)
        direction = np.asarray(direction, dtype=np.float64)
        norm = np.linalg.norm(direction)
        if norm == 0:
            raise ValueError('ray direction cannot be zero')
        direction = direction / norm

        bounds_min, bounds_max = self.origin, self.origin + self.size
        t0, t1 = 0.0, float(max_range)
        for axis in range(3):
            if abs(direction[axis]) < 1e-15:
                if origin[axis] < bounds_min[axis] or origin[axis] >= bounds_max[axis]:
                    return None
                continue
            a = (bounds_min[axis] - origin[axis]) / direction[axis]
            b = (bounds_max[axis] - origin[axis]) / direction[axis]
            t0 = max(t0, min(a, b))
            t1 = min(t1, max(a, b))
        if t1 < t0 or t1 < 0:
            return None

        t = max(t0, 0.0)
        p = origin + direction * (t + 1e-10)
        idx = np.floor((p - self.origin) / self.resolution).astype(int)
        idx = np.minimum(np.maximum(idx, 0), self.shape_xyz - 1)
        step = np.sign(direction).astype(int)
        t_delta = np.full(3, np.inf)
        t_max = np.full(3, np.inf)
        for axis in range(3):
            if step[axis] != 0:
                t_delta[axis] = self.resolution / abs(direction[axis])
                boundary = self.origin[axis] + (idx[axis] + (1 if step[axis] > 0 else 0)) * self.resolution
                t_max[axis] = t + (boundary - p[axis]) / direction[axis]

        while t <= t1 and np.all(idx >= 0) and np.all(idx < self.shape_xyz):
            if self.occupied[idx[2], idx[1], idx[0]]:
                return float(t)
            axis = int(np.argmin(t_max))
            t = t_max[axis]
            idx[axis] += step[axis]
            t_max[axis] += t_delta[axis]
        return None

    def save(self, path):
        np.savez_compressed(path, occupied=self.occupied, origin=self.origin,
                            size=self.size, resolution=self.resolution)


def load_mid360_directions(csv_path, downsample=2, samples=20000):
    """Return ordered Livox directions; ``samples=None`` loads the full pattern."""
    table = np.loadtxt(str(Path(csv_path)), delimiter=',', skiprows=1)
    if table.ndim != 2 or table.shape[1] != 3:
        raise ValueError('Expected Livox CSV columns Time/s, Azimuth/deg, Zenith/deg')
    if downsample < 1 or (samples is not None and samples < 1):
        raise ValueError('downsample and samples must be positive')
    if samples is None:
        samples = len(table)
    indices = np.arange(0, samples, downsample) % len(table)
    azimuth = np.deg2rad(table[indices, 1])
    vertical = np.deg2rad(table[indices, 2] - 90.0)
    directions = np.stack((np.cos(vertical) * np.cos(azimuth),
                           np.cos(vertical) * np.sin(azimuth),
                           -np.sin(vertical)), axis=1)
    return directions


def rotation_from_quaternion(x, y, z, w):
    """Quaternion (x,y,z,w) to rotation matrix."""
    n = x*x + y*y + z*z + w*w
    if n < 1e-15:
        return np.eye(3)
    s = 2.0 / n
    return np.array([
        [1-s*(y*y+z*z), s*(x*y-z*w), s*(x*z+y*w)],
        [s*(x*y+z*w), 1-s*(x*x+z*z), s*(y*z-x*w)],
        [s*(x*z-y*w), s*(y*z+x*w), 1-s*(x*x+y*y)],
    ])


def lidar_pose_from_body(position, body_rotation, pitch_degrees=23.5, z_offset=0.13):
    """World sensor origin and rotation for the current quadcopter_lidar URDF."""
    pitch = math.radians(pitch_degrees)
    body_to_lidar_rotation = np.array([
        [math.cos(pitch), 0.0, math.sin(pitch)],
        [0.0, 1.0, 0.0],
        [-math.sin(pitch), 0.0, math.cos(pitch)],
    ])
    origin = np.asarray(position, dtype=np.float64) + body_rotation @ np.array([0.0, 0.0, z_offset])
    return origin, body_rotation @ body_to_lidar_rotation
