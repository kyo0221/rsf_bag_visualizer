# rsf_bag_visualizer

RSF-X001（Hokuyo）の rosbag2 から、ナビゲーション観点で状態を確認する HTML レポートを生成します。
軌跡の連続性（ジャンプ）、GNSS/LIO の切替と復帰、不確かさ、LIO ドリフト、GNSS 品質、デバイス診断、データ欠落を
タブ切替で見られます。Plotly.js（CDN）でズーム・ホバーができます。

```bash
source /opt/ros/jazzy/setup.bash
python3 rsf_bag_visualizer.py ~/rosbag/rsf/tsukuba/tsukuba_full        # ./tsukuba_full_visualizer.html を生成して開く
python3 rsf_bag_visualizer.py <bag> --out report.html --no-open
python3 rsf_bag_visualizer.py <bag> --reextract                         # キャッシュ (~/.cache/rsf_bag_visualizer) を捨てて読み直す
python3 -m pytest tests
```

必要なもの: ROS 2 jazzy 環境（rosbag2_py, rclpy, nav_msgs, sensor_msgs, nmea_msgs, diagnostic_msgs）, numpy, PyYAML。

## タブ

| タブ | 内容 |
|---|---|
| 概要 | rsf_odom（Fix / Float / LIO / be careful 色分け）、LIO 生 2 種、UTM、生 GNSS（Fix）の重ね描き、ジャンプ、切替点 |
| rsf_odom（切替） | 色分け軌跡、速度、z、σ、yaw、rsf_odom_type / rsf_odom_state の帯 |
| LIO 生 | LiDAR レート / IMU レートの軌跡、z、σ、点群点数、GNSS との乖離・方位誤差・距離スケール |
| RSF 処理 LIO | rsf_odom の LIO 区間と純 LIO の重ね描き、|rsf_odom − 純 LIO|、進行方向差、GNSS との乖離 |
| GNSS | 品質別の生 GNSS 軌跡、UTM odom、gps_qual 帯、衛星数、HDOP、diff_age、σ、高度、GPRMC |
| 状態 | rsf_odom/fix の type/state、gps_qual、diagnostics（レベル・温度・CPU 等）、IMU、トピック受信間隔 |
| イベント・集計 | 要約、クラス別割合、Fix 復帰→odom 復帰の待ち時間、位置合わせ残差、イベント一覧（クリックで前後 ±30 s を表示） |

## 座標系

全 XY は `rsf_odom` 座標系です。生 GNSS（lat/lon → 局所 ENU）と UTM は、rsf_odom が GNSS モードかつ RTK Fix だった
サンプルで剛体位置合わせ（回転 + 並進）しています。「LIO を GNSS 基準に位置合わせ」を ON にすると、LIO 生は Fix が
得られた最初の 120 s で GNSS に合わせ込んだ座標になり、以降の乖離が LIO ドリフトとして読めます。

抽出時は 1 kHz トピックを 50 Hz、NavSatFix（切替出力）を 10 Hz に間引き、String は内容が変わった時だけデコードします。
位置ジャンプは 50 Hz サンプル間の移動量 1 m 超で検出します。
