import math

import numpy as np

CLASSES = ['fix', 'float', 'lio', 'lio_bc']
CLASS_LABEL = ['GNSS RTK Fix (gps_qual=4)', 'GNSS Float / DGPS (gps_qual=5/2)', 'LIO fallback (GNSS abnormal)',
               'LIO + state "be careful"']
CLASS_COLOR = ['#2a78d6', '#eb6834', '#1baf7a', '#4a3aa7']
INK2, MUTED = '#52514e', '#898781'
ALIGN_WINDOW_S = 120.0
JUMP_THRESH_M = 1.0


def rigid_align(src, dst):
    src = np.asarray(src, float)
    dst = np.asarray(dst, float)
    ca, cb = src.mean(0), dst.mean(0)
    u, _, vt = np.linalg.svd((src - ca).T @ (dst - cb))
    r = vt.T @ u.T
    if np.linalg.det(r) < 0:
        vt[-1] *= -1
        r = vt.T @ u.T
    return r, cb - r @ ca


def apply_rigid(r, t, xy):
    return (np.asarray(r) @ np.asarray(xy, float).T).T + np.asarray(t)


def enu_from_latlon(lat, lon, lat0, lon0):
    radius = 6378137.0
    e = np.radians(np.asarray(lon) - lon0) * math.cos(math.radians(lat0)) * radius
    n = np.radians(np.asarray(lat) - lat0) * radius
    return e, n


def runs_from_changes(changes, t_end):
    return [[float(t), float(changes[i + 1][0]) if i + 1 < len(changes) else float(t_end), v]
            for i, (t, v) in enumerate(changes)]


def value_index_at(change_t, t_query):
    return np.searchsorted(np.asarray(change_t, float), np.asarray(t_query, float), side='right') - 1


def detect_jumps(x, y, thresh):
    seg = np.hypot(np.diff(x), np.diff(y))
    return np.flatnonzero(seg > thresh), seg


def contiguous_runs(mask):
    idx = np.flatnonzero(mask)
    if len(idx) == 0:
        return []
    br = np.flatnonzero(np.diff(idx) > 1)
    return list(zip(np.r_[idx[0], idx[br + 1]], np.r_[idx[br], idx[-1]]))


def decimate_idx(t, rate):
    if len(t) < 2 or rate <= 0:
        return np.arange(len(t))
    step = max(1, int(round((len(t) - 1) / max(t[-1] - t[0], 1e-9) / rate)))
    idx = np.arange(0, len(t), step)
    return idx if idx[-1] == len(t) - 1 else np.r_[idx, len(t) - 1]


def course_deg(t, x, y, lag=1.0, min_move=0.3):
    dx = x - np.interp(t - lag, t, x)
    dy = y - np.interp(t - lag, t, y)
    c = np.degrees(np.arctan2(dy, dx))
    c[np.hypot(dx, dy) < min_move] = np.nan
    return c


def simulate_switch(lt, lx, ly, nt, G, fix, win=60.0, min_move=8.0, outage_s=2.0):
    te, GF = nt[fix], G[fix]
    LF = np.c_[np.interp(te, lt, lx), np.interp(te, lt, ly)]
    R = [rigid_align(LF, GF)[0]]
    for k in range(1, len(te)):
        sel = (te > te[k] - win) & (te <= te[k])
        R.append(rigid_align(LF[sel], GF[sel])[0] if sel.sum() >= 5 and np.hypot(*(GF[sel][-1] - GF[sel][0])) >= min_move else R[-1])
    R = np.array(R)
    k = np.clip(np.searchsorted(te, lt, side='right') - 1, 0, None)
    sim = GF[k] + np.einsum('nij,nj->ni', R[k], np.c_[lx, ly] - LF[k])
    sim[lt < te[0]] = np.nan
    mode = np.where(lt < te[0], -1, (lt - te[k] > 1.5).astype(int))
    pred = GF[:-1] + np.einsum('nij,nj->ni', R[:-1], LF[1:] - LF[:-1])
    corr = np.hypot(*(GF[1:] - pred).T)
    dt = np.diff(te)
    cum = np.r_[0.0, np.cumsum(np.hypot(np.diff(lx), np.diff(ly)))]
    jumps = [{'t': round(float(te[i + 1]), 1), 'outage_s': round(float(dt[i]), 1), 'jump_m': round(float(corr[i]), 2),
              'travel_m': round(float(np.interp(te[i + 1], lt, cum) - np.interp(te[i], lt, cum)), 1),
              'x0': round(float(pred[i, 0]), 3), 'y0': round(float(pred[i, 1]), 3),
              'x1': round(float(GF[i + 1, 0]), 3), 'y1': round(float(GF[i + 1, 1]), 3)} for i in np.flatnonzero(dt > outage_s)]
    return sim, mode, te[1:], corr, jumps, corr[dt <= outage_s]


def heading_windows(t, x, y, tg, xg, yg, win=10.0, min_move=2.0):
    out = [[], [], []]
    xi = np.interp(tg, t, x)
    yi = np.interp(tg, t, y)
    for a in np.arange(tg[0], tg[-1] - win, win):
        i = np.searchsorted(tg, a)
        j = min(np.searchsorted(tg, a + win), len(tg) - 1)
        if j <= i:
            continue
        dg = (xg[j] - xg[i], yg[j] - yg[i])
        do = (xi[j] - xi[i], yi[j] - yi[i])
        lg, lo = math.hypot(*dg), math.hypot(*do)
        if lg < min_move or lo < min_move:
            continue
        out[0].append(float(a + win / 2))
        out[1].append((math.degrees(math.atan2(do[1], do[0]) - math.atan2(dg[1], dg[0])) + 180) % 360 - 180)
        out[2].append(lo / lg)
    return [np.array(v) for v in out]


def find_roles(side):
    stored = side['stored']

    def by_type(kind):
        return [n for n, i in stored.items() if i['type'] == kind]

    def pick(cands, *hints):
        for h in hints:
            for n in cands:
                if h in n:
                    return n
        return None

    odoms = by_type('nav_msgs/msg/Odometry')
    strings = by_type('std_msgs/msg/String')
    navsats = by_type('sensor_msgs/msg/NavSatFix')
    roles = {
        'rsf_odom': pick(odoms, '/rsf_odom', 'switch_odom') or (odoms[0] if odoms else None),
        'lio_lidar': pick(odoms, 'lio_lidar', 'lidar_rate'),
        'lio_imu': pick(odoms, 'lio_imu', 'imu_rate'),
        'utm': pick(odoms, 'utm'),
        'navsat': pick(navsats, 'nav_sat_fix', 'navsat'),
        'rsf_fix': pick(navsats, '/rsf_fix', 'switch_fix'),
        'gpgga': (by_type('nmea_msgs/msg/Gpgga') or [None])[0],
        'gprmc': (by_type('nmea_msgs/msg/Gprmc') or [None])[0],
        'imu': (by_type('sensor_msgs/msg/Imu') or [None])[0],
        'cloud': (by_type('sensor_msgs/msg/PointCloud2') or [None])[0],
        'diag': (by_type('diagnostic_msgs/msg/DiagnosticArray') or [None])[0],
        'odom_type': pick(strings, 'odom_type'),
        'odom_state': pick(strings, 'odom_state'),
        'fix_type': pick(strings, 'fix_type'),
        'fix_state': pick(strings, 'fix_state'),
    }
    if roles['rsf_fix'] == roles['navsat']:
        roles['rsf_fix'] = None
    return roles


def cols(data, side, topic, *names):
    key = side['stored'][topic]['key']
    return [data[f'{key}__{n}'] for n in names]


def rounded(values, digits=3):
    return [None if v != v else round(float(v), digits) for v in np.asarray(values, float)]


def build(data, side, plot_rate=10.0):
    roles = find_roles(side)
    if roles['rsf_odom'] is None:
        raise SystemExit('no nav_msgs/Odometry topic in the bag')
    dur = float(side['duration_s'])
    report = {'meta': {'name': side['name'], 'bag_dir': side['bag_dir'], 'duration_s': dur, 't0_ns': side['t0_ns'],
                       'roles': roles, 'stored': side['stored'], 'frames': side.get('frames', {}),
                       'gaps': side.get('gaps', {}), 'jump_thresh': JUMP_THRESH_M, 'align_window': ALIGN_WINDOW_S},
              'sources': {}, 'rsf': {}, 'series': {}, 'strips': {}, 'metrics': {}, 'events': [], 'summary': {},
              'align': {}, 'sim': None}
    events = report['events']
    strings = side.get('strings', {})

    t, x, y, z, yaw, sx, sy = cols(data, side, roles['rsf_odom'], 't', 'x', 'y', 'z', 'yaw', 'sx', 'sy')

    def strip(role):
        changes = strings.get(roles[role] or '', [])
        return runs_from_changes(changes, dur), changes

    type_runs, type_changes = strip('odom_type')
    state_runs, state_changes = strip('odom_state')
    for role in ('odom_type', 'odom_state', 'fix_type', 'fix_state', 'diag'):
        report['strips'][role] = strip(role)[0]

    if roles['gpgga']:
        gt, gq, gs, gh, galt, gage = cols(data, side, roles['gpgga'], 't', 'qual', 'sats', 'hdop', 'alt', 'diff_age')
    else:
        gt = gq = gs = gh = galt = gage = np.array([])
    qual_changes = ([[float(gt[0]), int(gq[0])]] + [[float(gt[i + 1]), int(gq[i + 1])] for i in np.flatnonzero(np.diff(gq))]) if len(gt) else []
    report['strips']['gps_qual'] = runs_from_changes(qual_changes, dur)

    def active(changes, tq):
        if not changes:
            return np.array([''] * len(tq), dtype=object)
        vals = np.array([''] + [c[1] for c in changes], dtype=object)
        return vals[value_index_at([c[0] for c in changes], tq) + 1]

    def qual_at(tq):
        if not len(gt):
            return np.full(len(tq), -1)
        idx = value_index_at(gt, tq)
        return np.where(idx >= 0, gq[np.clip(idx, 0, len(gq) - 1)], -1).astype(int)

    typ = active(type_changes, t)
    sta = active(state_changes, t)
    q = qual_at(t)
    is_lio = np.array([s.startswith('LIO') for s in typ])
    careful = np.array([s == 'be careful' for s in sta])
    cls = np.zeros(len(t), dtype=int)
    cls[(~is_lio) & (q != 4) & (q != -1)] = 1
    cls[is_lio] = 2
    cls[is_lio & careful] = 3
    empty = np.array([s == '' for s in typ])
    if empty.any() and (~empty).any():
        cls[:int(np.argmax(~empty))] = cls[int(np.argmax(~empty))]

    jumps, seg = detect_jumps(x, y, JUMP_THRESH_M)
    path_cum = np.r_[0.0, np.cumsum(np.where(seg > JUMP_THRESH_M, 0.0, seg))]
    speed = path_cum - np.interp(t - 1.0, t, path_cum)

    def add_source(key, label, topic, color, dash, xa=None, ya=None, extra=None):
        tt, xx, yy, zz = cols(data, side, topic, 't', 'x', 'y', 'z')
        idx = decimate_idx(tt, plot_rate)
        src = {'label': label, 'topic': topic, 'color': color, 'dash': dash, 't': rounded(tt[idx], 2),
               'x': rounded(xx[idx]), 'y': rounded(yy[idx]), 'z': rounded(zz[idx], 2)}
        if xa is not None:
            src['xa'] = rounded(xa[idx])
            src['ya'] = rounded(ya[idx])
        for k, v in (extra or {}).items():
            src[k] = rounded(v[idx])
        report['sources'][key] = src
        return idx

    idx_r = add_source('rsf_odom', roles['rsf_odom'], roles['rsf_odom'], '#0b0b0b', 'solid',
                       extra={'speed': speed, 'sx': sx, 'sy': sy, 'yaw': np.degrees(yaw), 'course': course_deg(t, x, y)})
    report['rsf'] = {'cls': [int(v) for v in cls[idx_r]], 'cls_labels': CLASS_LABEL, 'cls_colors': CLASS_COLOR,
                     'jumps': [{'t': round(float(t[i + 1]), 2), 'x0': round(float(x[i]), 3), 'y0': round(float(y[i]), 3),
                                'x1': round(float(x[i + 1]), 3), 'y1': round(float(y[i + 1]), 3),
                                'len': round(float(seg[i]), 2), 'dz': round(float(z[i + 1] - z[i]), 2)} for i in jumps],
                     'transitions': []}
    for j in report['rsf']['jumps']:
        events.append({'t': j['t'], 'kind': 'jump', 'detail': f"位置ジャンプ {j['len']:.1f} m (dz {j['dz']:+.1f} m)", 'x': j['x1'], 'y': j['y1']})

    def add_transitions(runs, kind):
        prev = runs[0][2] if runs else None
        for k in range(1, len(runs)):
            t_s, t_e, v = runs[k]
            if (t_e - t_s < 2.0 and k < len(runs) - 1) or v == prev:
                continue
            i = min(np.searchsorted(t, t_s), len(t) - 1)
            item = {'t': round(float(t_s), 2), 'x': round(float(x[i]), 3), 'y': round(float(y[i]), 3), 'kind': kind,
                    'detail': f'{prev} → {v}'}
            report['rsf']['transitions'].append(item)
            events.append(dict(item))
            prev = v

    add_transitions(type_runs, 'odom_type')
    add_transitions(state_runs, 'odom_state')
    for k in range(1, len(qual_changes)):
        t_s, v = qual_changes[k]
        i = min(np.searchsorted(t, t_s), len(t) - 1)
        events.append({'t': round(t_s, 2), 'kind': 'gps_qual', 'detail': f'GPGGA quality {qual_changes[k - 1][1]} → {v}',
                       'x': round(float(x[i]), 3), 'y': round(float(y[i]), 3)})

    episodes = []
    lio_runs = contiguous_runs(is_lio)
    seen_runs = set()
    for k in range(1, len(qual_changes)):
        tq, vq = qual_changes[k]
        if vq != 4:
            continue
        i = min(np.searchsorted(t, tq), len(t) - 1)
        run = next((a for a, b in lio_runs if a <= i <= b), None)
        if run is None or run in seen_runs:
            continue
        seen_runs.add(run)
        later = np.flatnonzero(~is_lio[i:])
        back = float(t[i + later[0]]) if len(later) else None
        episodes.append({'t_fix': round(tq, 1), 't_back': None if back is None else round(back, 1),
                         'wait_s': None if back is None else round(back - tq, 1), 'until_end_s': round(dur - tq, 1)})
        events.append({'t': round(tq, 1), 'kind': 'fix_back', 'x': round(float(x[i]), 3), 'y': round(float(y[i]), 3),
                       'detail': 'GNSS fix 復帰 (odom は LIO のまま)' + (f' → {back - tq:.0f} s 後に GNSS へ' if back else ' → 終了まで復帰せず')})
    report['summary']['fix_back_episodes'] = episodes

    gnss = None
    if roles['navsat']:
        nt, lat, lon, alt, nstat, nsx = cols(data, side, roles['navsat'], 't', 'lat', 'lon', 'alt', 'status', 'sx')
        good = np.isfinite(lat) & (np.abs(lat) > 1e-6)
        if good.sum() > 10:
            e, n = enu_from_latlon(lat, lon, float(lat[good][0]), float(lon[good][0]))
            rx, ry = np.interp(nt, t, x), np.interp(nt, t, y)
            qn = qual_at(nt)
            cls_n = cls[np.clip(np.searchsorted(t, nt), 0, len(t) - 1)]
            mask, how = good & (cls_n == 0) & (qn == 4), 'GNSS-mode fix samples'
            if mask.sum() < 20:
                mask, how = good & (qn == 4), 'all fix samples'
            if mask.sum() < 5:
                mask, how = good, 'all samples'
            r, tr = rigid_align(np.c_[e[mask], n[mask]], np.c_[rx[mask], ry[mask]])
            G = apply_rigid(r, tr, np.c_[e, n])
            res = np.hypot(*(G[mask] - np.c_[rx[mask], ry[mask]]).T)
            report['align']['gnss'] = {'method': how, 'n': int(mask.sum()), 'rms_m': round(float(np.sqrt(np.mean(res ** 2))), 3),
                                       'rot_deg': round(math.degrees(math.atan2(r[1, 0], r[0, 0])), 3)}
            idx = decimate_idx(nt, plot_rate)
            report['sources']['gnss'] = {'label': roles['navsat'], 'topic': roles['navsat'], 'color': MUTED, 'dash': 'solid',
                                         't': rounded(nt[idx], 2), 'x': rounded(G[idx, 0]), 'y': rounded(G[idx, 1]),
                                         'z': rounded(alt[idx], 2), 'qual': [int(v) for v in qn[idx]], 'sigma': rounded(nsx[idx]),
                                         'status': [int(v) for v in nstat[idx]], 'lat': rounded(lat[idx], 7), 'lon': rounded(lon[idx], 7)}
            gnss = {'t': nt, 'G': G, 'fix': good & (qn == 4), 'rx': rx, 'ry': ry}

    if roles['utm']:
        ut, ux, uy, uz = cols(data, side, roles['utm'], 't', 'x', 'y', 'z')
        rx, ry = np.interp(ut, t, x), np.interp(ut, t, y)
        mask = cls[np.clip(np.searchsorted(t, ut), 0, len(t) - 1)] == 0
        if mask.sum() < 20:
            mask = np.ones(len(ut), bool)
        r, tr = rigid_align(np.c_[ux[mask], uy[mask]], np.c_[rx[mask], ry[mask]])
        U = apply_rigid(r, tr, np.c_[ux, uy])
        res = np.hypot(*(U[mask] - np.c_[rx[mask], ry[mask]]).T)
        report['align']['utm'] = {'n': int(mask.sum()), 'rms_m': round(float(np.sqrt(np.mean(res ** 2))), 3),
                                  'rot_deg': round(math.degrees(math.atan2(r[1, 0], r[0, 0])), 3)}
        idx = decimate_idx(ut, plot_rate)
        report['sources']['utm'] = {'label': roles['utm'], 'topic': roles['utm'], 'color': MUTED, 'dash': 'dash',
                                    't': rounded(ut[idx], 2), 'x': rounded(U[idx, 0]), 'y': rounded(U[idx, 1]), 'z': rounded(uz[idx], 2)}

    metrics = {}
    for key, dash in (('lio_lidar', 'solid'), ('lio_imu', 'dot')):
        if not roles[key]:
            continue
        lt, lx, ly, lz, lyaw, lsx, lsy, lsz = cols(data, side, roles[key], 't', 'x', 'y', 'z', 'yaw', 'sx', 'sy', 'sz')
        xa = ya = None
        if gnss is not None:
            nt, G = gnss['t'], gnss['G']
            lxi, lyi = np.interp(nt, lt, lx), np.interp(nt, lt, ly)
            mask = gnss['fix'] & (nt >= lt[0]) & (nt <= lt[-1])
            if mask.sum() >= 20:
                window = mask & (nt <= nt[mask][0] + ALIGN_WINDOW_S)
                if window.sum() < 20:
                    window = mask
                r, tr = rigid_align(np.c_[lxi[window], lyi[window]], G[window])
                A = apply_rigid(r, tr, np.c_[lx, ly])
                xa, ya = A[:, 0], A[:, 1]
                gap = np.hypot(*(apply_rigid(r, tr, np.c_[lxi, lyi]) - G).T)
                report['align'][key] = {'window_s': [round(float(nt[window][0]), 1), round(float(nt[window][-1]), 1)], 'n': int(window.sum()),
                                        'rms_m': round(float(np.sqrt(np.mean(gap[window] ** 2))), 3),
                                        'rot_deg': round(math.degrees(math.atan2(r[1, 0], r[0, 0])), 2)}
                ht, dh, ratio = heading_windows(lt, xa, ya, nt[mask], G[mask, 0], G[mask, 1])
                cum = np.r_[0, np.cumsum(np.hypot(np.diff(lx), np.diff(ly)))]
                metrics[key] = {'t': rounded(nt[mask], 1), 'gap': rounded(gap[mask], 2), 'dist': rounded(np.interp(nt[mask], lt, cum), 1),
                                'hdg_t': rounded(ht, 1), 'hdg_err': rounded(dh, 1), 'scale': rounded(ratio)}
        add_source(key, roles[key], roles[key], INK2, dash, xa, ya,
                   extra={'sx': lsx, 'sy': lsy, 'sz': lsz, 'yaw': np.degrees(lyaw), 'course': course_deg(lt, lx, ly)})

    if gnss is not None:
        nt, G, mask = gnss['t'], gnss['G'], gnss['fix']
        gap = np.hypot(*(G - np.c_[gnss['rx'], gnss['ry']]).T)
        ht, dh, ratio = heading_windows(t, x, y, nt[mask], G[mask, 0], G[mask, 1]) if mask.sum() > 20 else (np.array([]),) * 3
        metrics['rsf_odom'] = {'t': rounded(nt[mask], 1), 'gap': rounded(gap[mask], 2), 'hdg_t': rounded(ht, 1),
                               'hdg_err': rounded(dh, 1), 'scale': rounded(ratio)}
    if roles['lio_lidar']:
        lt, lx, ly = cols(data, side, roles['lio_lidar'], 't', 'x', 'y')
        offset = np.hypot(np.interp(lt, t, x) - lx, np.interp(lt, t, y) - ly)
        ht, dh, ratio = heading_windows(t, x, y, lt, lx, ly)
        idx = decimate_idx(lt, plot_rate)
        lio_mode = is_lio[np.clip(np.searchsorted(t, lt), 0, len(t) - 1)]
        metrics['rsf_vs_lio'] = {'t': rounded(lt[idx], 1), 'offset': rounded(offset[idx], 2), 'lio_mode': [int(v) for v in lio_mode[idx]],
                                 'hdg_t': rounded(ht, 1), 'hdg_diff': rounded(dh, 1), 'scale': rounded(ratio)}
        if gnss is not None and gnss['fix'].sum() >= 20:
            sim, mode, ct, corr, jumps, routine = simulate_switch(lt, lx, ly, gnss['t'], gnss['G'], gnss['fix'])
            report['sim'] = {'t': rounded(lt[idx], 2), 'x': rounded(sim[idx, 0]), 'y': rounded(sim[idx, 1]), 'mode': [int(v) for v in mode[idx]],
                             'corr_t': rounded(ct, 1), 'corr': rounded(corr, 2), 'jumps': jumps,
                             'routine_median_m': round(float(np.median(routine)), 3) if len(routine) else None,
                             'routine_p95_m': round(float(np.percentile(routine, 95)), 3) if len(routine) else None}
    report['metrics'] = metrics

    series = report['series']
    if len(gt):
        series['gps'] = {'t': rounded(gt, 1), 'qual': [int(v) for v in gq], 'sats': [int(v) for v in gs], 'hdop': rounded(gh, 2),
                         'alt': rounded(galt, 2), 'diff_age': rounded(gage, 1)}
    if roles['rsf_fix']:
        ft, fsx, falt = cols(data, side, roles['rsf_fix'], 't', 'sx', 'alt')
        idx = decimate_idx(ft, plot_rate)
        series['rsf_fix'] = {'t': rounded(ft[idx], 2), 'sigma': rounded(fsx[idx]), 'alt': rounded(falt[idx], 2)}
    if roles['cloud']:
        ct, cn = cols(data, side, roles['cloud'], 't', 'npts')
        idx = decimate_idx(ct, plot_rate)
        series['cloud'] = {'t': rounded(ct[idx], 2), 'npts': [int(v) for v in cn[idx]]}
    if roles['imu']:
        it, gx, gy, gz, ax, ay, az = cols(data, side, roles['imu'], 't', 'gx', 'gy', 'gz', 'ax', 'ay', 'az')
        idx = decimate_idx(it, plot_rate)
        series['imu'] = {'t': rounded(it[idx], 2), 'gyro_z': rounded(np.degrees(gz)[idx], 1),
                         'gyro': rounded(np.degrees(np.sqrt(gx ** 2 + gy ** 2 + gz ** 2))[idx], 1),
                         'acc': rounded(np.sqrt(ax ** 2 + ay ** 2 + az ** 2)[idx], 2)}
    if roles['gprmc']:
        pt, ps, ptr = cols(data, side, roles['gprmc'], 't', 'speed', 'track')
        series['gprmc'] = {'t': rounded(pt, 1), 'speed': rounded(ps, 2), 'track': rounded(ptr, 1)}
    if roles['diag']:
        key = side['stored'][roles['diag']]['key']
        names = sorted(k[len(key) + 2:] for k in data if k.startswith(key + '__') and k != key + '__t')
        dt = data[key + '__t']
        series['diag'] = {'t': rounded(dt, 1), **{n: rounded(data[f'{key}__{n}'], 2) for n in names}}

    path = float(np.sum(seg[seg <= JUMP_THRESH_M]))
    shares = {}
    for ci, c in enumerate(CLASSES):
        m = cls == ci
        shares[c] = {'label': CLASS_LABEL[ci], 'color': CLASS_COLOR[ci], 'share': round(float(m.mean() * 100), 1),
                     'dist_m': round(float(np.sum(seg[m[:-1] & (seg <= JUMP_THRESH_M)])), 1),
                     'time_s': round(float(np.sum(np.r_[np.diff(t), 0][m])), 1)}
    report['summary'].update({'duration_s': round(dur, 1), 'n_odom': side['stored'][roles['rsf_odom']]['count'], 'path_m': round(path, 1),
                              'jump_total_m': round(float(np.sum(seg[seg > JUMP_THRESH_M])), 1), 'n_jumps': int(len(jumps)),
                              'classes': shares, 'mean_speed': round(path / dur, 2), 'type_runs': type_runs,
                              'gps_qual_counts': {str(int(k)): int(v) for k, v in zip(*np.unique(gq, return_counts=True))} if len(gt) else {}})
    if 'lio_lidar' in metrics:
        gap = np.array([v for v in metrics['lio_lidar']['gap'] if v is not None])
        dist = np.array([v for v in metrics['lio_lidar']['dist'] if v is not None])
        if len(gap) > 5:
            report['summary']['lio_drift'] = {'gap_end_m': float(gap[-1]), 'gap_max_m': float(gap.max()), 'dist_end_m': float(dist[-1]),
                                              'gap_per_dist_pct': round(float(gap[-1] / max(dist[-1], 1) * 100), 2)}
    events.sort(key=lambda e: e['t'])
    return report
