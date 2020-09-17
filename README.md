# soma_ros_pkg
林研究室 林業用ロボット"SOMA"のROSパッケージ

#[外部パッケージ要求(Required packages)]
	hdl_graph_slam : LiDARに対応した３次元slamパッケージ
	ethzasl_xsens_driver : xsensのIMUを扱うパッケージ
	nmea_navsat_driver : USB接続のGPSを扱うパッケージ
	velodyne : velodyne社製LiDARを扱うパッケージ
	realsense-ros : R200,D435,D435iを扱うパッケージ

[ディレクトリ構成]
soma_ros_pkg : ルートディレクトリ(通称メタパッケージディレクトリ)
	└soma_ros : soma自律移動プログラム本体のディレクトリ
		├include
    |	└soma_ros : hファイル配置ディレクトリ
		|		├Behavior
		|		└Data
		|
		├launch : 実行用launchファイル配置ディレクトリ
		├rviz : 実行時のrvizコンフィグファイル配置ディレクトリ
		└src : cppファイル配置ディレクトリ
			├Behavior
    	└Data

[soma_rosディレクトリ内ファイルについて]
	CMakeLists.txt : catkin_makeするためのCMakeファイル
	nodelet_description.xml : 自作したnodeletの説明ファイル
  package.xml : soma_rosパッケージの依存関係の説明ファイル

