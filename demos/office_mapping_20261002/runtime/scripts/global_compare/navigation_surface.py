"""Measured local floor for visualization; never changes registered map points.

The body height is a search hint, not a fabricated ground measurement. A plane
requires spatially distributed real returns. Unsupported/multilevel locations
are explicitly masked instead of projecting an entire building into one slice.
"""
from dataclasses import dataclass
import numpy as np
from scipy.spatial import cKDTree


@dataclass
class GroundSurface:
    valid: bool
    reason: str
    floor_z: float
    coefficients: np.ndarray
    support_count: int
    support_xyz: np.ndarray
    base_xyz: np.ndarray

    @classmethod
    def from_scan(cls, xyz, base_xyz, hint_offset=.40):
        base = np.asarray(base_xyz, float)
        hint = float(base[2] - hint_offset)
        empty = np.empty((0, 3))
        invalid = lambda reason: cls(False, reason, hint, np.array([0., 0., hint]), 0, empty, base)
        points = np.asarray(xyz, float).reshape(-1, 3)
        points = points[np.isfinite(points).all(axis=1)]
        if not len(points):
            return invalid('no_returns')
        dist = np.linalg.norm(points[:, :2] - base[:2], axis=1)
        # Do not fit the robot body, the ceiling, or the floor several metres below.
        points = points[(dist > .38) & (dist < 3.5) & (points[:, 2] > hint - .55)
                        & (points[:, 2] < hint + .65)]
        if len(points) < 24:
            return invalid('insufficient_floor_returns')
        # One lowest real return per column prevents wall sampling density from
        # dominating RANSAC. No synthetic/densified points enter this fit.
        keys = np.floor(points[:, :2] / .15).astype(np.int64)
        order = np.argsort(points[:, 2], kind='stable')
        # Integer column keys avoid NumPy's structured axis sort over tens of
        # thousands of accumulated MID360 returns in the live 5 Hz callback.
        low=keys.min(axis=0);span=int(keys[:,1].max()-low[1])+1
        packed=(keys[:,0]-low[0])*span+(keys[:,1]-low[1])
        _, first = np.unique(packed[order], return_index=True)
        candidates = points[order[first]]
        if len(candidates) < 20:
            return invalid('insufficient_floor_columns')
        design = np.column_stack((candidates[:, :2] - base[:2], np.ones(len(candidates))))
        rng = np.random.default_rng(19)
        best = None
        best_count = 0
        for _ in range(100):
            ids = rng.choice(len(candidates), 3, replace=False)
            if abs(np.linalg.det(design[ids])) < .03:
                continue
            coefficients = np.linalg.solve(design[ids], candidates[ids, 2])
            if np.linalg.norm(coefficients[:2]) > .70 or abs(coefficients[2] - hint) > .30:
                continue
            mask = abs(design @ coefficients - candidates[:, 2]) < .055
            count = int(mask.sum())
            if count > best_count:
                support = candidates[mask]
                # A dense wall-bottom strip must not displace a less dense,
                # genuinely two-dimensional floor plane in the same scan.
                spread = np.linalg.eigvalsh(np.cov(support[:, :2].T)) if count > 3 else [0., 0.]
                if spread[0] < .025 or np.ptp(support[:, 0]) < .45 or np.ptp(support[:, 1]) < .45:
                    continue
                best, best_count = mask, count
        if best is None or best_count < 20 or best_count / len(candidates) < .18:
            return invalid('no_supported_floor_plane')
        support = candidates[best]
        spread = np.linalg.eigvalsh(np.cov(support[:, :2].T))
        if spread[0] < .025 or np.ptp(support[:, 0]) < .45 or np.ptp(support[:, 1]) < .45:
            return invalid('floor_support_not_spatially_distributed')
        coefficients = np.linalg.lstsq(design[best], support[:, 2], rcond=None)[0]
        floor = float(coefficients[2])
        coefficients[2] -= coefficients[:2] @ base[:2]
        return cls(True, 'measured_local_plane', floor, coefficients, len(support), support, base)

    def height_at(self, xy):
        xy = np.asarray(xy, float).reshape(-1, 2)
        z = np.column_stack((xy, np.ones(len(xy)))) @ self.coefficients
        if not self.valid:
            return z, np.zeros(len(xy), bool)
        distance, index = cKDTree(self.support_xyz[:, :2]).query(xy)
        # Within the observed plane footprint, preserve actual step/ramp returns
        # rather than extrapolating across gaps or into another floor.
        supported = (distance < .48) & (np.linalg.norm(xy - self.base_xyz[:2], axis=1) < 3.6)
        z[supported] = self.support_xyz[index[supported], 2]
        return z, supported

    def project(self, xyz, offset=.035, expected_tolerance=.65):
        points = np.asarray(xyz, float).reshape(-1, 3).copy()
        z, supported = self.height_at(points[:, :2])
        supported &= abs(points[:, 2] - z) <= expected_tolerance
        points[supported, 2] = z[supported] + offset
        return points, supported

    def metadata(self):
        return dict(valid=self.valid, reason=self.reason, floor_z=self.floor_z,
                    plane=self.coefficients.tolist(), support_count=self.support_count)

    def payload(self, source_stamp):
        return dict(self.metadata(), frame='lio_map', source_stamp=float(source_stamp),
                    support_xyz=self.support_xyz.tolist(), base_xyz=self.base_xyz.tolist(),
                    measurement_window_sec=2.)

    @classmethod
    def from_payload(cls, payload):
        if payload.get('frame') != 'lio_map':
            raise ValueError('Measured ground must be in the LIO frame')
        plane=np.asarray(payload['plane'],float).reshape(3)
        support=np.asarray(payload['support_xyz'],float).reshape(-1,3)
        base=np.asarray(payload['base_xyz'],float).reshape(3)
        if not (np.isfinite(plane).all() and np.isfinite(support).all() and np.isfinite(base).all()):
            raise ValueError('Non-finite measured ground')
        if payload['valid'] and len(support)<20:
            raise ValueError('Unsupported measured ground payload')
        return cls(bool(payload['valid']),str(payload['reason']),float(payload['floor_z']),
                   plane,len(support),support,base)
