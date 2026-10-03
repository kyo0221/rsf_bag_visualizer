#!/usr/bin/env python3
import argparse
import hashlib
import os
import sys
import webbrowser

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import analysis
import extract
import report


def cache_path(bag, cache_dir):
    meta = os.path.join(bag, 'metadata.yaml')
    seed = open(meta, 'rb').read() if os.path.exists(meta) else bag.encode()
    return os.path.join(cache_dir, f'{os.path.basename(bag)}-{hashlib.sha1(seed).hexdigest()[:10]}.npz')


def main():
    ap = argparse.ArgumentParser(description='RSF-X001 rosbag2 visualizer (HTML report)')
    ap.add_argument('bag', help='rosbag2 directory')
    ap.add_argument('--out', help='output html (default: ./<bag>_visualizer.html)')
    ap.add_argument('--cache', default=os.path.expanduser('~/.cache/rsf_bag_visualizer'))
    ap.add_argument('--reextract', action='store_true')
    ap.add_argument('--plot-rate', type=float, default=10.0, help='Hz kept for plotting')
    ap.add_argument('--no-open', action='store_true')
    a = ap.parse_args()
    bag = os.path.abspath(a.bag.rstrip('/'))
    if not os.path.isdir(bag):
        sys.exit(f'not a bag directory: {bag}')
    npz = cache_path(bag, a.cache)
    if a.reextract or not (os.path.exists(npz) and os.path.exists(npz + '.json')):
        print(f'extracting {bag}')
        extract.extract(bag, npz)
    data, side = extract.load(npz)
    out = a.out or f'{os.path.basename(bag)}_visualizer.html'
    report.write_html(analysis.build(data, side, a.plot_rate), out)
    print(f'wrote {out} ({os.path.getsize(out) / 1e6:.1f} MB)')
    if not a.no_open:
        webbrowser.open('file://' + os.path.abspath(out))


if __name__ == '__main__':
    main()
