# image_recognition study

画像認識に関する基礎的な知識を得るために，実践的にプログラムを試すためのパッケージである．

## 環境構築
- Ubuntu 20.04
- ROS Noetic

- カメラのポート割当により，カメラを起動することができるパッケージ
```bash
sudo apt install ros-noetic-usb-cam
```
詳しい使い方は[こちら](http://wiki.ros.org/usb_cam)

```bash
sudo apt install ros-noetic-image-view
```
詳しい使い方は[こちら](http://wiki.ros.org/image_view)

## ソースコード
### color_img_recog
- color_img_recog.py
https://qiita.com/keinko/items/c8f9390419cb1cbb6467\
サイトのサンプルコードを基準に，noeticのturtlesimで動くように，修正を加えたもの\
- 実行方法
```bash
roslaunch image_recognition_study color_img_recog.launch
```
カメラで取得した画像の中に青色を認識したら前進し，赤色を認識したら更新する．

## 利用したコマンド
- aptでinstallしたパッケージの確認
```bash
apt list --installed
```
- grepにより，表示されるパッケージ名をフィルタリング
```bash
apt list --installed | grep ros-noetic
```

- カメラのポート割当を出力する
```bash
ls /dev/video*
```

- 流れているトピック名を取得する
```bash
rostopic list
```

- rosでカメラを起動して，image_viewで表示する
以下，例を示す
```bash
roscore
---
rosrun usb_cam usb_cam_node _video_device:=/dev/video0
---
rosrun image_view image_view image:=/usb_cam/image_raw
```
なお，/dev/video1はカメラのポート番号，/usb_cam/image_rawは流れている画像のトピック名であるので，適宜変更する．

- rosのパッケージを検索する
以下，例を示す
```bash
rospack find cv_bridge
```
cv_bridgeがパッケージ名であるので，適宜変更する．

- rosのパッケージでpython3系に対応しているものの階層への移動
```bash
cd /opt/ros/noetic/lib/python3/dist-packages
```

- turtlesimの起動
参考リンク：http://wiki.ros.org/ja/ROS/Tutorials/UnderstandingTopics
```bash
roscore
---
rosrun turtlesim turtlesim_node
```

- turtlesimを矢印操作で動かすパッケージの起動
```bash
rosrun turtlesim turtle_teleop_key
```

- トピックをグラフで確認
```
rosrun rqt_graph rqt_graph
```

- turtlesimに送信しているトピックの型を出力する
```bash
rostopic type /turtle1/cmd_vel
```

## 参考リンク一覧
- ROSを使って画像情報でロボットを動かす / @keinko / Qiita
e
- ROSの勉強　第40弾：USBカメラ / @Yuya-Shimizu / Qiita
https://qiita.com/Yuya-Shimizu/items/10804c9ac44a3568d116