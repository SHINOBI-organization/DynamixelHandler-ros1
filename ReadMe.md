# DynamixelHandler-ros1

Robotis社の[Dynamixel](https://e-shop.robotis.co.jp/list.php?c_id=89)をROSから制御するための ros pkg `dynamixel_handler`を提供するリポジトリ.  

Dynamixelとやり取りを行うライブラリは[別のリポジトリ](https://github.com/SHINOBI-organization/lib_dynamixel)として管理しており，git submoduleの機能を使って取り込んでいる．

note: ROS1のみ対応

note: Dynamixel Xシリーズのみ対応（Pシリーズの対応は後ほど予定している）

## features of this package
 - node を起動するだけで連結したDynamixelを自動で認識
 - ROS topic の pub/sub のみでDynamixelを制御可能
   - 指定した周期で指定した情報を state topic としてpub (デフォルト: 電流/速度/位置, 50Hz)
   - 指定した周期でハードウェアエラーを error topic としてpub (デフォルト: 0.5Hz)
   - subした command topic に合わせて制御モードを自動で変更 (電流/速度/位置/電流制限付き位置/拡張位置制御に対応)
   - sub/pubされる情報はパルス値ではなく物理量
 - Serial通信の Raed/Write は ROS node の周期と同期
 - node を kill したタイミングで動作を停止
 - 初期化時にエラーを自動でクリア
 - エラークリア時の回転数消失問題をhoming offsetにより自動補正
 - ros param から各種 log 表示の制御が可能
   - Serial通信のエラー率
   - Serial通信の Read/Write にかかる平均時間
   - Read/Write されるパルス値
   - Readに失敗したID
   - etc...
     
## how to install

```
$ cd ~/catkin_ws/src
$ git clone https://github.com/SHINOBI-organization/DynamixelHandler-ros1 dynamixel_handler
$ cd ./dynamixel_handler
$ git submodule init
$ git submodule update
```

## how to use

### 1. Dynamixelを接続
   DynaimixelをディジーチェーンにしてUSBで接続する．

   note: baudrateがすべて同一かつ，idに重複がないように事前に設定すること
   
### 2. dynamixel_handlerの起動

baudrate: 1000000 かつ device name: /dec/ttyUSB0の場合

#### rosrun からの起動
argを指定してroscore, rosrunで実行

ターミナルを開いて以下を実行
```
$ roscore
```
別のターミナルを開いて以下を実行
```
$ rosrun dynamixel_handler dynamixel_handler_node _BAUDRATE:=1000000 DEVICE:=/dev/ttyUSB0
```


#### roslaunch からの起動
dynamixel_handler.launchのargにbaudrateとdevice_nameを設定し，roslaunchで起動する

dynamixel_handler.launchの以下の部分を編集し，保存
```xml
<arg name="DEVICE_NAME" default="/dev/ttyUSB0"/>
<arg name="BAUDRATE" default="1000000"/>
```
ターミナルを開いて以下を実行
```
$ roslaunch dynamixel_handler dynamixel_handler.launch
```

### 3. TopicのSub/Pub
ID:5のDynamixelを位置制御モード(position control mode)で角度を90degにしたい場合

角度の制御
```
$ rostopic pub /dynamixel/cmd/x/position dynamixel_handler/DynamixelCommand_X_ControlPosition "{id_list: [5], position__deg: [90]}" -1
```
ID:5のDynamixelが位置制御モードでなかった場合は自動で変換される．

"read_present_current", "read_present_velocity", "read_present_position" をtrueに設定した場合

状態の確認
```
$ rostopic echo /dyanmixel/state

# 出力例
---
stamp: 
  secs: 1703962959
  nsecs: 388530440
id_list: [5, 6]
current__mA: [0.0, -2.69]
velocity__deg_s: [0.0, 0.0]
position__deg: [89.91210937499999, -0.2636718750000023]
vel_trajectory__deg_s: []
pos_trajectory__deg: []
temperature__degC: []
input_voltage__V: []
---
stamp:
  secs: 1703962959
  nsecs: 396484578
id_list: [5, 6]
current__mA: [0.0, -2.69]
velocity__deg_s: [0.0, 0.0]
position__deg: [89.91210937499999, -0.2636718750000023]
vel_trajectory__deg_s: []
pos_trajectory__deg: []
temperature__degC: []
input_voltage__V: []
```
どの情報をpubするかは ros param から設定可能．
以下のparamを参照

## topic
#### Subscribed by dyanmixel_handler　
 - /dynamixel/cmd_free : 
 - /dynamixel/cmd/x/current
 - /dynamixel/cmd/x/velocity
 - /dynamixel/cmd/x/position
 - /dynamixel/cmd/x/extended_position
 - /dynamixel/cmd/x/current_position 
 - /dynamixel/cmd/option
 - /dynamixel/config/gain/w
 - /dynamixel/config/limit/w
 - /dynamixel/config/mode/w
 
#### Published from dyanmixel_handler　
 - /dynamixel/state_free
 - /dynamixel/state
 - /dynamixel/error
 - /dynamixel/config/gain/r
 - /dynamixel/config/limit/r
 - /dynamixel/config/mode/r

## param

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
 - temperature_limit  : 未実装，`/dynamixel/config/limit/w`のsubで設定し，現在値を`/dynamixel/config/limit/r`としてpubできるにようにする．
 - present_temperture : `/dynamixel/state`として, loop_rate/ratio_read_stateの周期でpubされる

### 電圧
 - max_voltage_limit     : 未実装，`/dynamixel/config/limit/w`のsubで設定し，現在値を`/dynamixel/config/limit/r`としてpubできるにようにする．
 - min_voltage_limit     : 未実装，`/dynamixel/config/limit/w`のsubで設定し，現在値を`/dynamixel/config/limit/r`としてpubできるにようにする．
 - present_input_voltage : `/dynamixel/state`として, loop_rate/ratio_read_stateの周期でpubされる

### PWM
 - pwm_limit   : 未実装，`/dynamixel/config/limit/w`のsubで設定し，現在値を`/dynamixel/config/limit/r`としてpubできるにようにする．
 - goal_pwm    : 未実装，`/dynamixel/cmd/x/pwm`をsubすると設定されるようにしたい
 - present_pwm : `/dynamixel/state`として, loop_rate/ratio_read_stateの周期でpubされる

### 電流
 - current_limit   : 未実装，`/dynamixel/config/limit/w`のsubで設定し，現在値を`/dynamixel/config/limit/r`としてpubできるにようにする．
 - goal_current    : `/dynamixel/cmd/x/current` or `/dynamixel/cmd/x/current_position`をsubすると設定され，loop_rateの周期で書き込まれる．
 - present_current : `/dynamixel/state`として, loop_rate/ratio_read_stateの周期でpubされる．

### 加速度
 - acceleration_limit   : 未実装，`/dynamixel/config/limit/w`のsubで設定し，現在値を`/dynamixel/config/limit/r`としてpubできるにようにする．
 - profile_acceleration : 未実装，`/dynamixel/cmd/option`をsubすると設定されるようにしたい．

### 速度
 - velocity_limit      : 未実装，`/dynamixel/config/limit/w`のsubで設定し，現在値を`/dynamixel/config/limit/r`としてpubできるにようにする．
 - profile_velocity    : 未実装，`/dynamixel/cmd/option`をsubすると設定されるようにしたい．
 - goal_velocity       : `/dynamixel/cmd/x/velocity`をsubすると設定され，loop_rateの周期で書き込まれる．
 - present_velocity    : `/dynamixel/state`として, loop_rate/ratio_read_stateの周期でpubされる．
 - velocity_trajectory : `/dynamixel/state`として, loop_rate/ratio_read_stateの周期でpubされる．

### 位置
 - max_position_limit    : 未実装，`/dynamixel/config/limit/w`のsubで設定し，現在値を`/dynamixel/config/limit/r`としてpubできるにようにする．
 - min_position_limit    : 未実装，`/dynamixel/config/limit/w`のsubで設定し，現在値を`/dynamixel/config/limit/r`としてpubできるにようにする．
 - goal_position         : `/dynamixel/cmd/x/position` or `/dynamixel/cmd/x/current_position` or `/dynamixel/cmd/x/extended_position` をsubすると設定され．loop_rateの周期で書き込まれる．
 - present_position      : `/dynamixel/state`として, loop_rate/ratio_read_stateの周期でpubされる．
 - position_trajectory   : `/dynamixel/state`として, loop_rate/ratio_read_stateの周期でpubされる．

### PIDパラメータ
 - velocity_i_gain       : 未実装，`/dynamixel/config/mode/w`のsubで設定し，現在値を`/dynamixel/config/mode/r`としてpubできるにようにする．
 - velocity_p_gain       : 未実装，`/dynamixel/config/mode/w`のsubで設定し，現在値を`/dynamixel/config/mode/r`としてpubできるにようにする．
 - position_d_gain       : 未実装，`/dynamixel/config/mode/w`のsubで設定し，現在値を`/dynamixel/config/mode/r`としてpubできるにようにする．
 - position_i_gain       : 未実装，`/dynamixel/config/mode/w`のsubで設定し，現在値を`/dynamixel/config/mode/r`としてpubできるにようにする．
 - position_p_gain       : 未実装，`/dynamixel/config/mode/w`のsubで設定し，現在値を`/dynamixel/config/mode/r`としてpubできるにようにする．
 - feedforward_acc_gain  : 未実装，`/dynamixel/config/mode/w`のsubで設定し，現在値を`/dynamixel/config/mode/r`としてpubできるにようにする．
 - feedforward_vel_gain  : 未実装，`/dynamixel/config/mode/w`のsubで設定し，現在値を`/dynamixel/config/mode/r`としてpubできるにようにする．
   
###  external port
 - external_port_mode_{x} : 未実装，topicから制御できるようにする．
 - external_port_data_{x} : 未実装，topicから制御できるようにする．

### 機能系
 - hardware_error_status  : `/dynamixel/error`としてloop_rate/ratio_read_errorの周期でpubされる. 
 - torque_enable          : 接続時に自動でトルクONされる. 
                            `/dynamixel/cmd_free`の`command='eneble'`でON,`command='disable'`でOFFに設定される．
 - operating_mode         : 対応するtopicのsubで自動で設定される．
                            未実装，現在値は`/dynamixel/config/mode/r`としてpubされるようにする． 
                            未実装，`/dynamixel/config/mode/w`をsubして設定されるようにする．
 - drive_mode             : 未実装，subで設定し，現在値を`/dynamixel/config/mode/r`としてpubできるにようにする．
                            未実装，`/dynamixel/config/mode/w`をsubして設定されるようにする．
 - homing_offset          : ユーザーは使用不可，reboot時の角度補正に用いられる．
 - bus_watchbdog          : 未実装，velocity control時に一定時間通信切れで自動停止するようにする．

### その他
 - moving_threshold       : not support
 - realtime_tick          : not support
 - registered_instruction : not support
 - shutdown               : not support
 - moving                 : not support
 - moving_status          : not support
 - led                    : not support
 - status_return_level    : not support, 常に2を前提とする
