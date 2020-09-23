# soma_ros_pkg
林研究室 林業用ロボット"SOMA"のROSパッケージ

## 外部パッケージ要求(Requirement)
* velodyne	: velodyne社製LiDARを扱うパッケージ
* realsense-ros : R200,D435,D435iを扱うパッケージ
* hdl_graph_slam	: LiDARに対応した３次元slamパッケージ
* ethzasl_xsens_driver	: xsensのIMUを扱うパッケージ
* nmea_navsat_driver	: USB接続のGPSを扱うパッケージ

## 外部パッケージ導入(Installation)
* velodyne
http://wiki.ros.org/velodyne  
for kinetic  
```
sudo apt install ros-kinetic-velodyne
```
for melodic  
```
sudo apt install ros-melodic-velodyne
```

* realsense-ros
https://github.com/IntelRealSense/realsense-ros#installation-instructions  

* for kinetic
```
sudo apt install ros-kinetic-realsense2-camera
```
* for melodic  
```
sudo apt install ros-melodic-realsense2-camera
```

* hdl_graph_slam
https://github.com/koide3/hdl_graph_slam

catkin_wsにソースコードをダウンロードしてcatkin_makeする

HDL Graph SLAMに必要なパッケージもインストールする必要がある

for kinetic
```
sudo apt-get install ros-kinetic-geodesy ros-kinetic-pcl-ros ros-kinetic-nmea-msgs ros-kinetic-libg2o
```
for melodic
```
sudo apt-get install ros-melodic-geodesy ros-melodic-pcl-ros ros-melodic-nmea-msgs ros-melodic-libg2o

cd catkin_ws/src
git clone https://github.com/koide3/ndt_omp.git
git clone https://github.com/koide3/hdl_graph_slam.git
cd ~/catkin_ws
catkin_make
```

ディレクトリ構成
soma_ros_pkg : ルートディレクトリ(通称メタパッケージディレクトリ)
└soma_ros : soma自律移動プログラム本体のディレクトリ
	├include
	|  └soma_ros : hファイル配置ディレクトリ
	|     ├Behavior
	|	    └Data
  |
	├launch : 実行用launchファイル配置ディレクトリ
	├rviz : 実行時のrvizコンフィグファイル配置ディレクトリ
	└src : cppファイル配置ディレクトリ
		├Behavior
		└Data

soma_rosディレクトリ内ファイルについて
 CMakeLists.txt : catkin_makeするためのCMakeファイル
 nodelet_description.xml : 自作したnodeletの説明ファイル
 package.xml : soma_rosパッケージの依存関係の説明ファイル

