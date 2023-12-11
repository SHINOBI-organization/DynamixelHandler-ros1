# DynamixelHandler-ros1

Robotis社の[Dynamixel](https://e-shop.robotis.co.jp/list.php?c_id=89)をROSから制御するための ros pkg `dynamixel_handler`を提供するリポジトリ.  

Dynamixelとやり取りを行うライブラリは[別のリポジトリ](https://github.com/SHINOBI-organization/lib_dynamixel)として管理しており，git submoduleの機能を使って取り込んでいる．

note: ROS1のみ対応

## how to install

```
$ cd ~/catkin_ws/src
$ git clone https://github.com/SHINOBI-organization/DynamixelHandler-ros1 dynamixel_handler
$ cd ./dynamixel_handler
$ git submodule init
$ git submodule update
```

## 初期設定と注意事項

`include\mylib_dynamixel\download\port_handler_linux.cpp` 内の `LATENCY_TIMER` と使用するUBSデバイスのlatency timerを一致させる必要がある．
このライブラリのデフォルトは`LATENCY_TIMER=16`で，USBデバイス側のデフォルトと等しいので通常は問題にならないが，高速化したいとき問題となる．

`include\download\port_handler_linux.cpp` 内の `LATENCY_TIMER`は，該当部分を書き換えてコンパイルでOK

使用するUSBデバイスのlatency timerは次のようにして変更する．
```
$ echo ACTION==\"add\", SUBSYSTEM==\"usb-serial\", DRIVER==\"ftdi_sio\", ATTR{latency_timer}=\"4\" > 99-dynamixelsdk-usb.rules
$ sudo cp ./99-dynamixelsdk-usb.rules /etc/udev/rules.d/
$ sudo udevadm control --reload-rules
$ sudo udevadm trigger --action=add
$ cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
```

一時的であれば以下のようにしてもよい．
```
$ echo 4 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
$ cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
```

コピペ用

```
echo ACTION==\"add\", SUBSYSTEM==\"usb-serial\", DRIVER==\"ftdi_sio\", ATTR{latency_timer}=\"4\" > 99-dynamixelsdk-usb.rules
sudo cp ./99-dynamixelsdk-usb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger --action=add
cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
```

```
echo 4 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
```