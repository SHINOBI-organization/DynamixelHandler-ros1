****# DynamixelHandler-ros1

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

## how to use

```

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

## Control Table との対応

### 温度
 - temperature_limit  : 未実装，launchから設定できるようにしたいけど．．．
 - present_temperture : 未実装，rostopicとしてpubされるようにしたい

### 電圧
 - max_voltage_limit     : 未実装，launchから設定できるようにしたいけど．．．
 - min_voltage_limit     : 未実装，launchから設定できるようにしたいけど．．．
 - present_input_voltage : 未実装，rostopicとしてpubされるようにしたい

### PWM
 - pwm_limit   : 未実装，launchから設定できるようにしたいけど．．．
 - goal_pwm    : 未実装，特定の型のトピックのcallbackで設定されるようにしたい 
 - present_pwm : 未実装，rostopicとしてpubされるようにしたい        

### 電流
 - current_limit   : 未実装，launchから設定できるようにしたいけど．．．
 - goal_current    : 未実装，特定の型のトピックのcallbackで設定されるようにしたい 
 - present_current : 未実装，rostopicとしてpubされるようにしたい

### 加速度
 - acceleration_limit   : 未実装，launchから設定できるようにしたいけど．．．
 - profile_acceleration : 未実装， goal position/velocity書き込み時にオプションとしてつけたい

### 速度
 - velocity_limit      : 未実装，launchから設定できるようにしたいけど．．．
 - goal_velocity       : 未実装，特定の型のトピックのcallbackで設定されるようにしたい 
 - present_velocity    : 未実装，rostopicとしてpubされるようにしたい
 - profile_velocity    : 未実装， goal position書き込み時にオプションとしてつけたい
 - velocity_trajectory : not support

### 位置
 - max_position_limit    : 未実装，launchから設定できるようにしたいけど．．．これはかなり必要
 - min_position_limit    : 未実装，launchから設定できるようにしたいけど．．．これはかなり必要
 - goal_position         : /dynamixel/cmdをsubすると設定される．write周期は全体ループと同期
                           todo 特定の型のトピックのcallbackで設定されるようにしたい 
 - present_position      : rostopicとしてpubされる. read & pub周期は全体ループと同期
 - position_trajectory   : not support

### PIDパラメータ
 - velocity_i_gain       : 未実装，launchから設定できるようにしたい, RAMの値なので需要はある
 - velocity_p_gain       : 未実装，launchから設定できるようにしたい, RAMの値なので需要はある
 - position_d_gain       : 未実装，launchから設定できるようにしたい, RAMの値なので需要はある
 - position_i_gain       : 未実装，launchから設定できるようにしたい, RAMの値なので需要はある
 - position_p_gain       : 未実装，launchから設定できるようにしたい, RAMの値なので需要はある
 - feedforward_acc_gain  : 未実装，launchから設定できるようにしたい, RAMの値なので需要はある
 - feedforward_vel_gain  : 未実装，launchから設定できるようにしたい, RAMの値なので需要はある

###  external port
 - external_port_mode_{x} : 未実装，topicから制御できるようにする 
 - external_port_data_{x} : 未実装，topicから制御できるようにする 

### 機能
 - operating_mode         : 未実装，subしたtopicの型に合わせて自動で変わるようにする．
 - drive_mode             : not support 常に000000000を前提としたい．
 - homing_offset          : ユーザーは使用不可，reboot時の角度補正に用いる
 - hardware_error_status  : error_ratioの割合でエラーのチェックを回す. 
                            todo error 検出はstatus packetに任せて詳細のpubに用いる
 - torque_enable          : 接続時に自動でトルクONされる. 
                            todo topicからon/off, 初期状態の設定
 - bus_watchbdog          : 未実装，velocity control時に一定時間通信切れで自動停止
 - moving_threshold       : not support
 - realtime_tick          : not support
 - registered_instruction : not support
 - shutdown               : not support
 - moving                 : not support
 - moving_status          : not support
 - led                    : not support
 - status_return_level    : not support, 常に2を前提とする