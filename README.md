# rsf_bag_visualizer

RSF-X001（Hokuyo）の rosbag2 をブラウザで解析する ROS 2 パッケージです。bag を置いたフォルダを launch で指定すると
一覧ページが開き、bag を選ぶと解析してレポート（軌跡、GNSS/LIO 切替、LIO 生 / RSF 処理 LIO、GNSS、デバイス状態、イベント）を表示します。

```bash
cd ~/rsf_ws && colcon build --packages-select rsf_bag_visualizer --symlink-install && source install/setup.bash
ros2 launch rsf_bag_visualizer visualizer.launch.py                      # bag_dir:=~/rsf_ws/rosbag, port:=8765, open_browser:=true
ros2 launch rsf_bag_visualizer visualizer.launch.py bag_dir:=/path/to/bags
```

- `bag_dir` 直下の各フォルダ（`metadata.yaml` を含む rosbag2 ディレクトリ）が一覧に出ます。
- 「表示」で解析してレポートを開きます。抽出キャッシュ（npz）とレポート（html）は `~/.cache/rsf_bag_visualizer` に保存され、2 回目以降は数秒で開きます。
  「再解析」は bag を読み直します（10 分の bag で約 30 s、40 分で約 4 分）。
- レポートの描画には Plotly.js（CDN）を使うため、閲覧時にネットワークが必要です。

## タブ

| タブ | 内容 |
|---|---|
| 概要 | rsf_odom（Fix / Float / LIO / be careful 色分け）、LIO 生 2 種、UTM、生 GNSS（Fix）の重ね描き、ジャンプ、切替点 |
| rsf_odom（切替） | 色分け軌跡、速度、z、σ、yaw、rsf_odom_type / rsf_odom_state の帯 |
| LIO 生 | LiDAR レート / IMU レートの軌跡、z、σ、点群点数、GNSS との乖離・方位誤差・距離スケール |
| RSF 処理 LIO | rsf_odom の LIO 区間と純 LIO の重ね描き、|rsf_odom − 純 LIO|、進行方向差、GNSS との乖離 |
| GNSS | 品質別の生 GNSS 軌跡、UTM odom、gps_qual 帯、衛星数、HDOP、diff_age、σ、高度、GPRMC |
| 状態 | rsf_odom/fix の type/state、gps_qual、diagnostics、IMU、トピック受信間隔 |
| イベント・集計 | 要約、クラス別割合、Fix 復帰→odom 復帰の待ち時間、位置合わせ残差、イベント一覧（クリックで前後 ±30 s を表示） |

## 座標系

全 XY は `rsf_odom` 座標系です。生 GNSS（lat/lon → 局所 ENU）と UTM は、rsf_odom が GNSS モードかつ RTK Fix だった
サンプルで剛体位置合わせ（回転 + 並進）しています。「LIO を GNSS 基準に位置合わせ」を ON にすると、LIO 生は Fix が
得られた最初の 120 s で GNSS に合わせ込んだ座標になり、以降の乖離が LIO ドリフトとして読めます。

抽出時は 1 kHz トピックを 50 Hz、NavSatFix（切替出力）を 10 Hz に間引き、String は内容が変わった時だけデコードします。
位置ジャンプは 50 Hz サンプル間の移動量 1 m 超で検出します。

## 構成

`rsf_bag_visualizer/extract.py`（bag → npz）、`analysis.py`（分類・位置合わせ・指標・イベント）、`report.py`（HTML 生成）、
`server.py`（一覧・解析ジョブ・配信）、`launch/visualizer.launch.py`。テストは `python3 -m pytest tests`。
