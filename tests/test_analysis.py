import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from rsf_bag_visualizer import analysis as A


def test_rigid_align_recovers_transform():
    rng = np.random.default_rng(0)
    src = rng.normal(size=(200, 2)) * 50
    th = math.radians(37.0)
    rot = np.array([[math.cos(th), -math.sin(th)], [math.sin(th), math.cos(th)]])
    shift = np.array([12.5, -3.0])
    dst = (rot @ src.T).T + shift + rng.normal(scale=0.01, size=src.shape)
    r, t = A.rigid_align(src, dst)
    assert np.allclose(r, rot, atol=1e-3) and np.allclose(t, shift, atol=0.05)
    assert np.hypot(*(A.apply_rigid(r, t, src) - dst).T).max() < 0.1


def test_rigid_align_is_a_proper_rotation():
    src = np.array([[0, 0], [1, 0], [0, 1], [1, 1.0]])
    r, _ = A.rigid_align(src, src[:, ::-1].copy())
    assert np.linalg.det(r) > 0.99


def test_runs_and_value_index():
    changes = [[0.0, 'GNSS'], [10.0, 'LIO'], [20.0, 'GNSS']]
    assert A.runs_from_changes(changes, 30.0) == [[0.0, 10.0, 'GNSS'], [10.0, 20.0, 'LIO'], [20.0, 30.0, 'GNSS']]
    assert list(A.value_index_at([0.0, 10.0, 20.0], [-1, 0, 5, 10, 15, 25])) == [-1, 0, 0, 1, 1, 2]


def test_jumps_and_runs():
    jumps, seg = A.detect_jumps(np.array([0, 0.1, 0.2, 5.0, 5.1]), np.zeros(5), 1.0)
    assert list(jumps) == [2] and abs(seg[2] - 4.8) < 1e-9
    assert A.contiguous_runs(np.array([0, 1, 1, 0, 1], bool)) == [(1, 2), (4, 4)]


def test_decimate_idx_keeps_endpoints():
    t = np.linspace(0, 100, 100001)
    idx = A.decimate_idx(t, 10.0)
    assert idx[0] == 0 and idx[-1] == len(t) - 1 and 1000 <= len(idx) <= 1002


def test_enu_scale():
    e, n = A.enu_from_latlon(np.array([36.0, 36.0009]), np.array([140.0, 140.0]), 36.0, 140.0)
    assert abs(n[1] - 100.0) < 1.0 and abs(e[1]) < 1e-6


def test_heading_windows_detects_rotation():
    t = np.linspace(0, 100, 1001)
    xg, yg = t * 1.0, np.zeros_like(t)
    th = math.radians(30)
    ht, dh, ratio = A.heading_windows(t, xg * math.cos(th), xg * math.sin(th), t, xg, yg, win=10.0)
    assert len(ht) > 5 and np.allclose(dh, 30.0, atol=0.5) and np.allclose(ratio, 1.0, atol=1e-6)


def test_find_roles_defaults():
    side = {'stored': {
        '/rsf/rsf_odom': {'type': 'nav_msgs/msg/Odometry'}, '/rsf/lio_lidar_rate_odom': {'type': 'nav_msgs/msg/Odometry'},
        '/rsf/lio_imu_rate_odom': {'type': 'nav_msgs/msg/Odometry'}, '/rsf/utm_coord_odom': {'type': 'nav_msgs/msg/Odometry'},
        '/rsf/nav_sat_fix': {'type': 'sensor_msgs/msg/NavSatFix'}, '/rsf/rsf_fix': {'type': 'sensor_msgs/msg/NavSatFix'},
        '/rsf/rsf_odom_type': {'type': 'std_msgs/msg/String'}, '/rsf/rsf_odom_state': {'type': 'std_msgs/msg/String'},
        '/rsf/gpgga': {'type': 'nmea_msgs/msg/Gpgga'}, '/rsf/diagnostics': {'type': 'diagnostic_msgs/msg/DiagnosticArray'}}}
    roles = A.find_roles(side)
    assert roles['rsf_odom'] == '/rsf/rsf_odom' and roles['lio_lidar'] == '/rsf/lio_lidar_rate_odom'
    assert roles['lio_imu'] == '/rsf/lio_imu_rate_odom' and roles['utm'] == '/rsf/utm_coord_odom'
    assert roles['navsat'] == '/rsf/nav_sat_fix' and roles['rsf_fix'] == '/rsf/rsf_fix'
    assert roles['odom_type'] == '/rsf/rsf_odom_type' and roles['diag'] == '/rsf/diagnostics'


def test_course_deg_direction_and_stationary():
    t = np.linspace(0, 20, 201)
    c = A.course_deg(t, t * 1.0, t * 1.0)
    assert np.allclose(c[20:], 45.0) and np.isnan(A.course_deg(t, np.zeros_like(t), np.zeros_like(t))).all()


def test_simulate_switch_jump_equals_lio_error():
    lt = np.linspace(0, 100, 2001)
    lx, ly = lt * 1.0, np.zeros_like(lt)
    nt = np.arange(0, 100, 0.1)
    G = np.c_[nt * 1.0, np.full_like(nt, 5.0)]
    fix = ~((nt > 40) & (nt < 60))
    lx = lx + np.where(lt > 40, 0.1 * np.minimum(lt - 40, 20), 0.0)
    sim, mode, ct, corr, jumps, routine = A.simulate_switch(lt, lx, ly, nt, G, fix)
    assert len(jumps) == 1 and abs(jumps[0]['jump_m'] - 2.0) < 0.15 and abs(jumps[0]['outage_s'] - 20.0) < 0.2
    assert mode[np.searchsorted(lt, 50.0)] == 1 and mode[np.searchsorted(lt, 20.0)] == 0
    assert np.nanmax(np.abs(sim[lt < 40, 1] - 5.0)) < 1e-6 and np.median(routine) < 0.01
