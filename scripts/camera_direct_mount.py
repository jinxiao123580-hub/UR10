#!/usr/bin/env python3
"""Offline CAD-only camera/bracket separation. All mesh coordinates are CAD mm.

This is an assembly proposal, never a hand-eye transform or a motion command.
The rectangular housing retains the dimensions of the supplied CAD proxy.
"""
import argparse
import json
from pathlib import Path
import struct

import numpy as np

DTYPE = np.dtype([('normal', '<f4', 3), ('vertices', '<f4', (3, 3)), ('attr', '<u2')])


def read_stl(path):
    raw = Path(path).read_bytes()
    count = struct.unpack_from('<I', raw, 80)[0]
    return np.frombuffer(raw, dtype=DTYPE, count=count, offset=84)['vertices'].astype(float)


def write_stl(path, triangles):
    triangles = np.asarray(triangles, dtype=float)
    records = np.zeros(len(triangles), dtype=DTYPE)
    normals = np.cross(triangles[:, 1]-triangles[:, 0], triangles[:, 2]-triangles[:, 0])
    length = np.linalg.norm(normals, axis=1)
    records['normal'] = normals/length[:, None]
    records['vertices'] = triangles
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    Path(path).write_bytes(b'CAD direct camera mount; model only'.ljust(80, b' ') +
                          struct.pack('<I', len(records)) + records.tobytes())


def direct_camera(camera_path, connector_path):
    source = read_stl(camera_path)
    points = np.unique(source.reshape(-1, 3), axis=0)
    # Camera and bracket share an edge, so a connected-component split would
    # keep them together. Identify the cuboid by its original 85/50/130-mm edges.
    left = points[np.isclose(points[:, 0], points[:, 0].min(), atol=1e-4)]
    corner = left[np.argmin(left[:, 2])]
    bottom_points = points[np.isclose(points[:, 2], corner[2], atol=1e-4)]
    edges = bottom_points-corner
    u = edges[np.argmin(abs(np.linalg.norm(edges, axis=1)-85.))]
    v = edges[np.argmin(abs(np.linalg.norm(edges, axis=1)-50.))]
    corners = np.array([corner+i*u+j*v+[0, 0, k*130.] for i in (0, 1) for j in (0, 1) for k in (0, 1)])
    from scipy.spatial import ConvexHull
    if not np.all(np.linalg.norm(corners[:, None]-points[None, :], axis=-1).min(axis=1)<1e-3):
        raise ValueError('Expected all eight original camera corners in the CAD union')
    hull = ConvexHull(corners)
    housing = corners[hull.simplices].copy()
    # Recreate the shared mounting face removed by the original CAD boolean
    # union. Orient the closed cuboid outwards for STL rendering.
    for triangle in housing:
        normal = np.cross(triangle[1]-triangle[0], triangle[2]-triangle[0])
        if normal @ (triangle.mean(axis=0)-corners.mean(axis=0)) < 0:
            triangle[[1, 2]] = triangle[[2, 1]]
    vertices = np.unique(housing.reshape(-1, 3), axis=0)
    centre = vertices.mean(axis=0)
    # The supplied cuboid's 85-mm edge lies at -30 degrees in CAD XY.
    # Recover the exact angle from its bottom face rather than approximate clipping.
    bottom = vertices[np.isclose(vertices[:, 2], vertices[:, 2].min(), atol=1e-4)]
    start = bottom[np.argmin(bottom[:, 0])]
    vectors = bottom-start
    width = vectors[np.argmin(abs(np.linalg.norm(vectors, axis=1)-85.))]
    angle = -np.arctan2(width[1], width[0]); c, s = np.cos(angle), np.sin(angle)
    rotation = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.]])
    aligned = (housing-centre) @ rotation.T
    size = np.ptp(aligned.reshape(-1, 3), axis=0)
    if not np.allclose(size, [85., 50., 130.], atol=.002):
        raise ValueError('Unexpected original CAD housing dimensions: %s' % size)
    # Latest operator photo: long edge HORIZONTAL. The broad 130 x 85 face
    # touches the connector side, with only the 50-mm thickness protruding.
    broad_face_to_connector = np.array([[0., -1., 0], [1., 0, 0], [0, 0, 1.]])
    aligned = aligned @ broad_face_to_connector.T
    size = np.ptp(aligned.reshape(-1, 3), axis=0)
    connector = read_stl(connector_path).reshape(-1, 3)
    face = connector[np.isclose(connector[:, 0], connector[:, 0].min(), atol=1e-4)]
    mounting_point = (face.min(0)+face.max(0))/2
    # Align with the upper surface at the CAMERA attachment end, excluding the
    # 13-mm raised circular flange seat nearer the force sensor. The
    # camera hangs below it rather than protruding above the connector.
    new_centre = np.array([mounting_point[0]-size[0]/2,
                           face[:, 1].max()-size[1]/2, mounting_point[2]])
    result = aligned+new_centre
    report = {
        'status': 'nominal_CAD_assembly_only_not_handeye_calibration',
        'camera_model': 'Mech-Eye PRO XS',
        'orientation_source': 'operator photo bdec90dcd17b73f71f712a0edbcf25c6.jpg and correction on 2026-10-27',
        'housing_size_cad_xyz_mm': size.tolist(),
        'size_source': 'existing CAD cuboid; not independently measured product dimensions',
        'source_triangle_count': len(source),
        'camera_triangle_count': len(housing),
        'intermediate_bracket_removed': True,
        'split_method': 'recover eight existing housing corners; close shared CAD boolean-union mounting face',
        'mounting_point_cad_mm': mounting_point.tolist(),
        'housing_centre_cad_mm': new_centre.tolist(),
        'camera_top_cad_y_mm': float(result[:, :, 1].max()),
        'connector_top_cad_y_mm': float(face[:, 1].max()),
        'excluded_sensor_seat_top_cad_y_mm': float(connector[:, 1].max()),
        'top_flush_error_mm': float(result[:, :, 1].max()-face[:, 1].max()),
        'contact_face_size_mm': [130.0, 85.0],
        'contact_face_description': 'broad 130 x 85 face against connector side; 50 mm thickness projects outward',
        'long_edge_direction_cad': [0, 0, 1],
        'long_edge_direction_tool0': [-1, 0, 0],
        'optical_direction_cad': [0, -1, 0],
        'optical_direction_tool0': [0, 0, 1],
        'orientation_scope': 'long edge transverse to brush/tool0 +Z, horizontal in the illustrated writing posture; optical direction nominal only',
        'mounting_assumption': 'housing +X broad face (130 x 85 mm) flush against retained connector minimum-X side; camera top coplanar with connector camera-end top surface, not raised sensor seat; CAD Z centres aligned; 130-mm long edge horizontal; screw pattern not represented',
        'handeye_updated': False, 'robot_motion_sent': False,
    }
    return result, report


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source-dir', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--report', type=Path)
    a = p.parse_args()
    camera = next(a.source_dir.glob('*3D相机*STL'))
    connector = next(a.source_dir.glob('*连接1*STL'))
    triangles, report = direct_camera(camera, connector)
    write_stl(a.output, triangles)
    if a.report:
        a.report.parent.mkdir(parents=True, exist_ok=True)
        a.report.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(report, ensure_ascii=False))


if __name__ == '__main__':
    main()
