import argparse
import hashlib
import html
import json
import os
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, unquote, urlparse

import yaml

from . import analysis, extract, report

STYLE = '''body{margin:0;background:#f9f9f7;color:#0b0b0b;font:14px/1.5 system-ui,-apple-system,"Segoe UI",sans-serif}
main{max-width:1100px;margin:0 auto;padding:24px 16px}h1{font-size:20px;font-weight:600;margin:0 0 4px}
.sub{color:#52514e;margin-bottom:16px}table{border-collapse:collapse;width:100%;background:#fcfcfb;border:1px solid #e1e0d9;border-radius:8px}
th,td{text-align:left;padding:8px 12px;border-bottom:1px solid #e1e0d9;font-variant-numeric:tabular-nums}th{color:#52514e;font-weight:600}
a.btn{display:inline-block;padding:4px 12px;border:1px solid #c3c2b7;border-radius:6px;background:#fcfcfb;color:#0b0b0b;text-decoration:none;margin-right:6px}
a.btn.primary{background:#2a78d6;border-color:#2a78d6;color:#fff}.state{color:#52514e}.wait{padding:40px 16px;text-align:center}
.bar{height:6px;background:#e1e0d9;border-radius:3px;overflow:hidden;margin:16px auto;max-width:480px}.bar div{height:100%;width:30%;background:#2a78d6;animation:slide 1.2s infinite}
@keyframes slide{0%{margin-left:0}100%{margin-left:70%}}pre{color:#52514e;font:12px/1.4 ui-monospace,monospace}'''

INDEX = '''<!doctype html><html lang="ja"><head><meta charset="utf-8"><title>rsf_bag_visualizer</title><style>{style}</style></head>
<body><main><h1>rsf_bag_visualizer</h1><div class="sub">{bag_dir}</div>
<table><tr><th>bag</th><th>記録開始</th><th>長さ</th><th>サイズ</th><th>状態</th><th></th></tr>{rows}</table>
<p class="sub">bag ディレクトリ（metadata.yaml を含むフォルダ）を上のフォルダに置くと一覧に出ます。レポートと抽出キャッシュは {cache_dir} に保存されます。</p></main></body></html>'''

WAIT = '''<!doctype html><html lang="ja"><head><meta charset="utf-8"><title>{name} - 解析中</title><style>{style}</style></head>
<body><main><div class="wait"><h1>{name} を解析しています</h1><div class="bar"><div></div></div>
<div id="msg" class="sub">開始しました</div><pre id="log"></pre><a class="btn" href="/">一覧へ戻る</a></div></main>
<script>
const t0=Date.now();
async function poll(){{const r=await fetch('/status/{name}');const s=await r.json();
document.getElementById('msg').textContent=`${{Math.round((Date.now()-t0)/1000)}} s 経過`;document.getElementById('log').textContent=s.log.join('\\n');
if(s.state==='ready')location.replace('/report/{name}');else if(s.state.startsWith('error'))document.getElementById('msg').textContent=s.state;else setTimeout(poll,1500);}}
poll();</script></body></html>'''


def list_bags(bag_dir):
    bags = []
    for name in sorted(os.listdir(bag_dir)):
        path = os.path.join(bag_dir, name)
        meta = os.path.join(path, 'metadata.yaml')
        if not os.path.isfile(meta):
            continue
        info = yaml.safe_load(open(meta))['rosbag2_bagfile_information']
        size = sum(os.path.getsize(os.path.join(path, f)) for f in os.listdir(path))
        bags.append({'name': name, 'duration_s': info['duration']['nanoseconds'] / 1e9,
                     'start': time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(info['starting_time']['nanoseconds_since_epoch'] / 1e9)),
                     'size_gb': size / 1e9, 'meta_mtime': os.path.getmtime(meta)})
    return bags


class Jobs:
    def __init__(self, bag_dir, cache_dir):
        self.bag_dir = bag_dir
        self.cache_dir = cache_dir
        self.lock = threading.Lock()
        self.jobs = {}
        os.makedirs(cache_dir, exist_ok=True)

    def report_path(self, name):
        return os.path.join(self.cache_dir, name + '.html')

    def npz_path(self, name):
        meta = os.path.join(self.bag_dir, name, 'metadata.yaml')
        return os.path.join(self.cache_dir, f'{name}-{hashlib.sha1(open(meta, "rb").read()).hexdigest()[:10]}.npz')

    def state(self, name):
        with self.lock:
            job = self.jobs.get(name)
        if job and job['state'] != 'ready':
            return job
        html_path = self.report_path(name)
        meta = os.path.join(self.bag_dir, name, 'metadata.yaml')
        if os.path.isfile(html_path) and os.path.getmtime(html_path) >= os.path.getmtime(meta):
            return {'state': 'ready', 'log': []}
        return job or {'state': 'none', 'log': []}

    def start(self, name, rebuild=False):
        with self.lock:
            if name in self.jobs and self.jobs[name]['state'] == 'running':
                return
            self.jobs[name] = {'state': 'running', 'log': []}
        threading.Thread(target=self.run, args=(name, rebuild), daemon=True).start()

    def run(self, name, rebuild):
        log = self.jobs[name]['log']
        try:
            bag = os.path.join(self.bag_dir, name)
            npz = self.npz_path(name)
            if rebuild or not (os.path.exists(npz) and os.path.exists(npz + '.json')):
                log.append('bag を読み込んでいます')
                extract.extract(bag, npz, progress=lambda m: log.append(m.strip()))
            else:
                log.append('抽出キャッシュを使用')
            log.append('レポートを生成しています')
            data, side = extract.load(npz)
            report.write_html(analysis.build(data, side), self.report_path(name))
            self.jobs[name]['state'] = 'ready'
        except Exception as e:
            self.jobs[name]['state'] = f'error: {e!r}'


def make_handler(jobs):
    class Handler(BaseHTTPRequestHandler):
        def send(self, code, body, ctype='text/html; charset=utf-8'):
            data = body.encode('utf-8') if isinstance(body, str) else body
            self.send_response(code)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            url = urlparse(self.path)
            parts = [unquote(p) for p in url.path.strip('/').split('/') if p]
            query = parse_qs(url.query)
            if not parts:
                return self.index()
            if len(parts) == 2 and parts[0] in ('view', 'status', 'report'):
                name = os.path.basename(parts[1])
                if not os.path.isfile(os.path.join(jobs.bag_dir, name, 'metadata.yaml')):
                    return self.send(404, 'bag not found', 'text/plain; charset=utf-8')
                if parts[0] == 'status':
                    return self.send(200, json.dumps(jobs.state(name)), 'application/json')
                if parts[0] == 'report':
                    if jobs.state(name)['state'] != 'ready':
                        return self.send(404, 'report not ready', 'text/plain; charset=utf-8')
                    return self.send(200, open(jobs.report_path(name), 'rb').read())
                rebuild = query.get('rebuild', ['0'])[0] == '1'
                if not rebuild and jobs.state(name)['state'] == 'ready':
                    return self.send(200, open(jobs.report_path(name), 'rb').read())
                jobs.start(name, rebuild)
                return self.send(200, WAIT.format(style=STYLE, name=html.escape(name)))
            self.send(404, 'not found', 'text/plain; charset=utf-8')

        def index(self):
            rows = []
            for b in list_bags(jobs.bag_dir):
                st = jobs.state(b['name'])['state']
                label = {'ready': '解析済み', 'running': '解析中', 'none': '未解析'}.get(st, st)
                n = html.escape(b['name'])
                rows.append(f'<tr><td>{n}</td><td>{b["start"]}</td><td>{b["duration_s"]:.0f} s</td><td>{b["size_gb"]:.2f} GB</td>'
                            f'<td class="state">{html.escape(label)}</td><td><a class="btn primary" href="/view/{n}">表示</a><a class="btn" href="/view/{n}?rebuild=1">再解析</a></td></tr>')
            if not rows:
                rows.append('<tr><td colspan="6">bag がありません</td></tr>')
            self.send(200, INDEX.format(style=STYLE, bag_dir=html.escape(jobs.bag_dir), rows=''.join(rows), cache_dir=html.escape(jobs.cache_dir)))

        def log_message(self, fmt, *args):
            pass
    return Handler


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--bag-dir', default=os.path.expanduser('~/rsf_ws/rosbag'))
    ap.add_argument('--cache-dir', default=os.path.expanduser('~/.cache/rsf_bag_visualizer'))
    ap.add_argument('--port', type=int, default=8765)
    ap.add_argument('--open-browser', default='true')
    a = ap.parse_args()
    bag_dir = os.path.abspath(os.path.expanduser(a.bag_dir))
    os.makedirs(bag_dir, exist_ok=True)
    server = ThreadingHTTPServer(('127.0.0.1', a.port), make_handler(Jobs(bag_dir, a.cache_dir)))
    url = f'http://127.0.0.1:{a.port}/'
    print(f'rsf_bag_visualizer: {url}  bags: {bag_dir}', flush=True)
    if a.open_browser.lower() in ('1', 'true', 'yes'):
        threading.Timer(0.5, webbrowser.open, args=(url,)).start()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
