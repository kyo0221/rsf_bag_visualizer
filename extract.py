import json
import math
import os
import time

import numpy as np

RATES = {'nav_msgs/msg/Odometry': 50.0, 'sensor_msgs/msg/NavSatFix': 10.0, 'sensor_msgs/msg/Imu': 50.0}
HANDLED = set(RATES) | {'nmea_msgs/msg/Gpgga', 'nmea_msgs/msg/Gprmc', 'sensor_msgs/msg/PointCloud2',
                        'std_msgs/msg/String', 'diagnostic_msgs/msg/DiagnosticArray'}


def topic_key(name):
    return name.strip('/').replace('/', '__')


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def read_metadata(bag_dir):
    import rosbag2_py
    import yaml
    storage = 'mcap'
    path = os.path.join(bag_dir, 'metadata.yaml')
    if os.path.exists(path):
        storage = yaml.safe_load(open(path))['rosbag2_bagfile_information'].get('storage_identifier', storage)
    md = rosbag2_py.Info().read_metadata(bag_dir, storage)
    topics = {t.topic_metadata.name: {'type': t.topic_metadata.type, 'count': int(t.message_count)}
              for t in md.topics_with_message_count}
    return {'storage': storage, 't0_ns': int(md.starting_time.nanoseconds),
            'duration_s': md.duration.nanoseconds / 1e9, 'topics': topics}


def numeric(text):
    try:
        return float(text)
    except ValueError:
        return None


def extract(bag_dir, out_npz, progress=print):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    bag_dir = os.path.abspath(bag_dir)
    meta = read_metadata(bag_dir)
    t0 = meta['t0_ns']
    duration = max(meta['duration_s'], 1e-3)
    decim = {}
    for name, info in meta['topics'].items():
        if info['type'] in HANDLED and info['count']:
            target = RATES.get(info['type'])
            decim[name] = max(1, int(round(info['count'] / duration / target))) if target else 1
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_dir, storage_id=''), rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    wanted = [n for n in decim if n in types]
    reader.set_filter(rosbag2_py.StorageFilter(topics=wanted))
    classes = {n: get_message(types[n]) for n in wanted}

    cols = {n: {} for n in wanted}
    strings = {n: [] for n in wanted if types[n] in ('std_msgs/msg/String', 'diagnostic_msgs/msg/DiagnosticArray')}
    last_bytes = {}
    last_ts = {}
    gaps = {n: {'max_s': 0.0, 'over_1s': []} for n in wanted}
    frames = {}
    seen = {n: 0 for n in wanted}
    total = 0
    started = time.time()
    report_at = 500000

    def push(topic, **values):
        for key, value in values.items():
            cols[topic].setdefault(key, []).append(value)

    while reader.has_next():
        topic, data, ts = reader.read_next()
        total += 1
        if total >= report_at:
            progress(f'  {total} msgs, {time.time() - started:.0f} s')
            report_at += 500000
        seen[topic] += 1
        t = (ts - t0) / 1e9
        if topic in last_ts:
            gap = (ts - last_ts[topic]) / 1e9
            if gap > gaps[topic]['max_s']:
                gaps[topic]['max_s'] = gap
            if gap > 1.0 and len(gaps[topic]['over_1s']) < 100:
                gaps[topic]['over_1s'].append([round(t, 2), round(gap, 2)])
        last_ts[topic] = ts
        kind = types[topic]
        if kind == 'std_msgs/msg/String':
            if last_bytes.get(topic) != data:
                last_bytes[topic] = data
                strings[topic].append([round(t, 4), deserialize_message(data, classes[topic]).data])
            continue
        if (seen[topic] - 1) % decim[topic]:
            continue
        m = deserialize_message(data, classes[topic])
        if kind == 'nav_msgs/msg/Odometry':
            p = m.pose.pose.position
            c = m.pose.covariance
            frames.setdefault(topic, [m.header.frame_id, m.child_frame_id])
            push(topic, t=t, x=p.x, y=p.y, z=p.z, yaw=yaw_of(m.pose.pose.orientation),
                 sx=math.sqrt(max(c[0], 0.0)), sy=math.sqrt(max(c[7], 0.0)), sz=math.sqrt(max(c[14], 0.0)),
                 syaw=math.sqrt(max(c[35], 0.0)))
        elif kind == 'sensor_msgs/msg/NavSatFix':
            c = m.position_covariance
            frames.setdefault(topic, [m.header.frame_id, ''])
            push(topic, t=t, lat=m.latitude, lon=m.longitude, alt=m.altitude, status=float(m.status.status),
                 sx=math.sqrt(max(c[0], 0.0)), sz=math.sqrt(max(c[8], 0.0)))
        elif kind == 'nmea_msgs/msg/Gpgga':
            push(topic, t=t, qual=float(m.gps_qual), sats=float(m.num_sats), hdop=float(m.hdop),
                 alt=float(m.alt), diff_age=float(m.diff_age))
        elif kind == 'nmea_msgs/msg/Gprmc':
            push(topic, t=t, speed=float(m.speed), track=float(m.track))
        elif kind == 'sensor_msgs/msg/Imu':
            w = m.angular_velocity
            a = m.linear_acceleration
            push(topic, t=t, gx=w.x, gy=w.y, gz=w.z, ax=a.x, ay=a.y, az=a.z)
        elif kind == 'sensor_msgs/msg/PointCloud2':
            push(topic, t=t, npts=float(m.width * m.height))
        elif kind == 'diagnostic_msgs/msg/DiagnosticArray':
            if not m.status:
                continue
            s = m.status[0]
            level = int.from_bytes(s.level, 'little') if isinstance(s.level, bytes) else int(s.level)
            text = f'{s.name} level={level} {s.message}'
            if not strings[topic] or strings[topic][-1][1] != text:
                strings[topic].append([round(t, 4), text])
            values = {kv.key: numeric(kv.value) for kv in s.values}
            push(topic, t=t, **{k: v for k, v in values.items() if v is not None})

    arrays = {}
    stored = {}
    for topic in wanted:
        key = topic_key(topic)
        if cols[topic]:
            n = len(cols[topic]['t'])
            for col, values in cols[topic].items():
                if len(values) == n:
                    arrays[f'{key}__{col}'] = np.asarray(values, dtype=np.float64)
        stored[topic] = {'type': types[topic], 'key': key, 'decim': decim[topic],
                         'count': meta['topics'][topic]['count'],
                         'rate_hz': round(meta['topics'][topic]['count'] / duration, 2),
                         'n': len(cols[topic].get('t', [])) or len(strings.get(topic, []))}
    side = {'bag_dir': bag_dir, 'name': os.path.basename(bag_dir.rstrip('/')), 'storage': meta['storage'],
            't0_ns': t0, 'duration_s': meta['duration_s'], 'all_topics': meta['topics'], 'stored': stored,
            'frames': frames, 'strings': strings, 'gaps': gaps, 'rates': RATES,
            'extracted_at': time.strftime('%Y-%m-%dT%H:%M:%S'), 'read_seconds': round(time.time() - started, 1)}
    os.makedirs(os.path.dirname(os.path.abspath(out_npz)) or '.', exist_ok=True)
    np.savez(out_npz, **arrays)
    json.dump(side, open(out_npz + '.json', 'w'), ensure_ascii=False)
    progress(f'  {total} msgs in {time.time() - started:.0f} s -> {out_npz}')
    return side


def load(npz_path):
    return dict(np.load(npz_path)), json.load(open(npz_path + '.json'))
