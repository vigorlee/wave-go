"""Read the verified 923 RGB/intensity PCD without changing its coordinates."""
import numpy as np

PCD_DTYPE = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
                      ('rgb', '<u4'), ('intensity', '<u2')])


def read_pcd(path):
    with path.open('rb') as stream:
        header = []
        while True:
            line = stream.readline()
            if not line:
                raise ValueError('PCD header has no DATA binary line')
            header.append(line.decode('ascii').strip())
            if line.strip() == b'DATA binary':
                break
        if 'FIELDS x y z rgb intensity' not in header or 'SIZE 4 4 4 4 2' not in header:
            raise ValueError('Expected the verified 923 RGB/intensity PCD layout')
        points = np.fromfile(stream, dtype=PCD_DTYPE)
    width = int(next(line.split()[1] for line in header if line.startswith('POINTS ')))
    if len(points) != width:
        raise ValueError(f'PCD truncation: header={width}, read={len(points)}')
    return points
