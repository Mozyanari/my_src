
先に、移動ベース用のUSBシリアルケーブルを接続  /dev/ttyUSB0　として認識させる。	なし
次に、パンチルト用のUSBシリアルケーブルを接続  /dev/ttyUSB1　として認識させる。	黄タグ

LANケーブルを接続　PCのIPアドレスを、192.168.0.5 
　　　　　　　　　(センサがデフォルトで、192.168.0.10なので、5でなくても、10以外なら可)
$ sudo chmod a+rw /dev/ttyUSB0
$ sudo chmod a+rw /dev/ttyUSB1

$ cd ~/catkin_ws/src/test_scan_subs/launch
$ roslaunch TonFenceLinePan2.launch 
$ rosrun rviz rviz		my_frame
encorder
URG 型番調べる UTM 30LX EW
前の柵と左右の柵色分ける
simplerecorder iphone iphone simplerecorder
