# はじめに
本パッケージは，MSEとの共同研究開発で進めた自動伐倒機のソフトウェアになります．

# 実行方法（How to use）
1. __センサノード起動__  
LiDAR（VLP-16），IMU（MTi-30）をまとめて起動する
```
roslaunch soma_mse sensor.launch
```
- __Arguments__ (default value)  
  - __base_frame_id__ (base_link) : ロボット本体のframe名
  - __gui__ (true) : rviz表示フラグ

2. __SLAMノード起動__
LiDARベースSLAMであるHDL Graph SLAMを使う
```
roslaunch soma_mse hdl_graph_slam.launch
```

3. __Harvesting Behavior ノード起動__  
