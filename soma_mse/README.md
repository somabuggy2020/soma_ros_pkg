# はじめに
本パッケージは，MSEとの共同研究開発で進めた自動伐倒機のソフトウェアになります．

# 実行方法（How to use）
1. __センサノード起動__  
LiDAR（VLP-16），IMU（MTi-30）をまとめて起動する
```
roslaunch soma_mse sensor.launch
```
2. __SLAMノード起動__
LiDARベースSLAMであるHDL Graph SLAMを使う
```
roslaunch soma_mse hdl_graph_slam.launch
