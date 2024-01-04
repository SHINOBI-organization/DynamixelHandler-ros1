# DynamixelHandler-ros1

Robotis社の[Dynamixel](https://e-shop.robotis.co.jp/list.php?c_id=89)をROSから制御するための ros pkg `dynamixel_handler`を提供するリポジトリ.  

Dynamixelとやり取りを行うライブラリは[別のリポジトリ](https://github.com/SHINOBI-organization/lib_dynamixel)として管理しており，git submoduleの機能を使って取り込んでいる．

note: ROS1のみ対応

note: Dynamixel Xシリーズのみ対応（Pシリーズの対応は後ほど予定している）

## features of this package
 - Dynamixelというサーボを動かすことに特化した最小単位のPkg
   - このPkgの dynamixel_handler node と ロボットの制御を行う別の node を組み合わせて使う
   - ほかのSDKと異なりコードを直接編集する必要なし
   - ほかのSDKと異なりコントロールテーブルのaddressや分解能などは意識する必要なし
   - 動かしたいDynamixelが「Jointとして使われているのか」「Wheelとして使われているのか」などの事前情報は不要
 - ROS topic の pub/sub のみでDynamixelを制御可能
   - 指定した周期で指定した情報を state topic としてpub (デフォルト: 電流/速度/位置, 50Hz)
   - 指定した周期でハードウェアエラーを error topic としてpub (デフォルト: 0.5Hz)
   - subした command topic に合わせて制御モードを自動で変更 (PWM制御は非対応)
   - sub/pubされる情報はパルス値ではなく物理量
   - ユーザは純粋にサーボの目標状態を指令するだけでサーボを制御でき，  
     また，同様にサーボの現在状態を受け取れる
 - 比較的高速なRead/Writeを実現 (12サーボに電流/速度/位置を Read/Write しても150Hzくらいは出せる)
   - 複数のアドレスを一括で読み書き & 複数のサーボを同時に読み書き(SyncRead/SyncWrite) によって Serial通信の回数を抑える
   - Fast Sync Read インストラクションを活用して Read を高速化
   - Raed/Write は ROS node の周期と同期しているので，topicのcallbackに左右されない．
   - ※ 適切な`LATENCY_TIMER`(2 ~ 4msくらい)と`baudrate`(1000000 ~ 推奨)の設定が必要
 - 開発における面倒を減らす機能
    - 初期化時に連結したDynamixelを自動で認識
    - 初期化時にエラーを自動でクリア (Optional)
    - 初期化時にトルクを自動でON (Optional)
    - node を kill したタイミングで動作を停止 
    - node を kill したタイミングでトルクをOFF (Optional)
    - エラークリア時の回転数消失問題を homing offset により自動補正
    - baudrateを一括で変更可能 (別nodeとして独立) 
 - ros param から各種 log 表示の制御が可能
   - Read/Write にかかる平均時間とSerial通信の成功率
   - Read/Write されるパルス値
   - Readに失敗したID
   - etc...

***************************

## how to install

```
$ cd ~/catkin_ws/src
$ git clone https://github.com/SHINOBI-organization/DynamixelHandler-ros1 dynamixel_handler
$ cd ./dynamixel_handler
$ git submodule init
$ git submodule update
```

***************************

## how to use

### 1. Dynamixelを接続
DynaimixelをディジーチェーンにしてUSBで接続する．
idに重複がないように事前にDynamixel Wizardなどを用いて設定すること.
baudrateを変更したい場合は次のlaunchファイルからdynamixel_unify_baudrate nodeを実行する.

dynamixel_unify_baudrate.launchの以下の部分を編集し，保存
``` xml
<!-- dynamixel_unify_baudrate.launch -->
<arg name="DEVICE_NAME" default="/dev/ttyUSB0"/>
<arg name="TARGET_BAUDRATE" default="1000000"/>
```
ターミナルを開いて次を実行
```
roslaunch dynamixel_handler dynamixel_unify_baudrate.launch
```
全てのdynamixelのbaudrateを`TARGET_BAUDRATE`に設定してくれる．変更が終わると自動でnodeは終了する．

### 2. dynamixel_handler nodeの起動

baudrate: 1000000 かつ device name: /dec/ttyUSB0の場合

dynamixel_handler.launchのargにbaudrateとdevice_nameを設定し，roslaunchで起動する

dynamixel_handler.launchの以下の部分を編集し，保存
```xml
<!-- dynamixel_handler.launch -->
<arg name="DEVICE_NAME" default="/dev/ttyUSB0"/>
<arg name="BAUDRATE" default="1000000"/>
```
ターミナルを開いて以下を実行
```
$ roslaunch dynamixel_handler dynamixel_handler.launch
```
連結したDynamixelを探索し，見つかったDynamixelの初期設定を行う．
通信状態によっては連結しているのに見つからない場合もあるので，その場合はros paramの `dyn_comm/retry_num` を大きくする．
`init/expected_servo_num`に0以上が設定されている場合，その個数のDynamixelが見つからないと初期化失敗で自動でnodeが終了する．

### 3. Dynamixelを制御

ID:5のDynamixel Xシリーズ のサーボを位置制御モード(position control mode)で角度を90degにしたい場合．

以下のように，`/dynamixel/cmd/x/position` topicにIDと角度を設定してpublishすればよい．

```
$ rostopic pub /dynamixel/cmd/x/position \
dynamixel_handler/DynamixelCommand_X_ControlPosition \
"{id_list: [5], position__deg: [90]}" -1
```
ID:5のDynamixelが位置制御モードでなかった場合は自動で変換される．

#### command topic 

制御指令はカスタム msg として次のように定義している．

位置制御, `/dynamixel/cmd/x/position`に対応
```yml
# DynamixelCommnad_X_ControlPosition.msg
uint16[] id_list
float64[] position__deg
```

速度制御, `/dynamixel/cmd/x/velocity`に対応
```yml
# DynamixelCommnad_X_ControlVelocity.msg
uint16[] id_list
float64[] velocity__deg_s
```

電流制御, `/dynamixel/cmd/x/current`に対応
```yml
# DynamixelCommnad_X_ControlCurrent.msg
uint16[] id_list
float64[] current__mA
```

拡張位置制御用，`/dynamixel/cmd/x/extended_position`に対応
```yml
# DynamixelCommnad_X_ControlExtendedPosition.msg
uint16[] id_list
float64[] position__deg
float64[] rotation # optional, 256までの回転数を指定できる
```

電流制限付き位置制御用，`/dynamixel/cmd/x/current_position`に対応
```yml
# DynamixelCommnad_X_ControlCurrentPosition.msg
uint16[] id_list
float64[] current__mA
float64[] position__deg
float64[] rotation # optional, 256までの回転数を指定できる
```
note: topic監視によるデバックの容易性の観点から角度はすべてdegにしてある

### 4. Dynamixelの情報を取得

ID:5とID:6のモータが接続している場合

`/dyanmixel/state` topic として一定周期で raed & pubされ続けている．
周期はros paramが `loop_rate=100` かつ `ratio/state_read=2`の時 100/2 = 50Hzとなる．

```
$ rostopic echo /dyanmixel/state
```

出力例
```yml
---
stamp: 
  secs: 1703962959
  nsecs: 388530440
id_list: [5, 6] # 認識されているサーボのID
current__mA: [0.0, -2.69] # 現在の電流値
velocity__deg_s: [0.0, 0.0] # 現在の各速度
position__deg: [89.91210937499999, -0.2636718750000023] # 現在の角度
vel_trajectory__deg_s: [] # 目標速度 みたいなもの
pos_trajectory__deg: [] # 目標角度 みたいなもの
temperature__degC: [] # 現在の温度
input_voltage__V: [] # 現在の入力電圧
---
```
どの情報をpubするかは ros param から設定可能．

dynamixelからのread方式は Sync Read であり，すべてのIDから一斉にreadするようになっている．
ただし，ros param `use/fast_read=true` の場合は  Fast Sync Read が用いられる．

また，上記の出力例にあるように複数の情報を読み込んでいるが，複数情報を一括でreadするか，分割でreadするかは，
ros param `use/split_read` によって変更できる．
分割でreadする場合は，読み込む情報の数分だけreadに時間がかかるので注意．

***************************

## sample code

### cpp
```
```

### python
```
```

***************************

## topic

詳細は[メッセージの定義](https://github.com/SHINOBI-organization/DynamixelHandler-ros1/tree/main/msg)を参照

#### Subscribed by dyanmixel_handler　

サーボへの入力を行うためのtopic.

 - `/dynamixel/command` (`DynamixelCommand` type) :   
 dynamixelの起動や停止，エラー解除コマンドなどを送るためのtopic 
 - `/dynamixel/cmd/x/current` (`DynamixelCommand_X_ControlCurrent` type) :   
 電流制御モードで動かすためのtopic
 - `/dynamixel/cmd/x/velocity` (`DynamixelCommand_X_ControlVelocity` type) :   
 速度制御モードで動かすためのtopic
 - `/dynamixel/cmd/x/position` (`DynamixelCommand_X_ControlPosition` type) :   
 位置制御モードで動かすためのtopic
 - `/dynamixel/cmd/x/extended_position` (`DynamixelCommand_X_ControlExtendedPosition` type) :   
 拡張位置制御モードで動かすためのtopic
 - `/dynamixel/cmd/x/current_position ` (`DynamixelCommand_X_ControlCurrentPosition` type) :   
 電流制限付き位置制御モードで動かすためのtopic
 - `/dynamixel/cmd/profile` (`DynamixelCommand_Profile` type) :   
 profile_accelerationとprofile_velocityを設定するためのtopic
 - `/dynamixel/option/gain/w` (`DynamixelOption_Gain` type) : 未実装
 - `/dynamixel/option/limit/w` (`DynamixelOption_Limit` type) : 未実装
 - `/dynamixel/option/mode/w` (`DynamixelOption_Mode` type)  : 未実装
 
#### Published from dyanmixel_handler　

サーボからの出力を監視するためのtopic.

 - /dynamixel/state
 - /dynamixel/error
 - /dynamixel/option/gain/r : 未実装
 - /dynamixel/option/limit/r
 - /dynamixel/option/mode/r : 未実装

***************************

## param

```xml
<!-- 通信の設定 -->
<param name="device_name" value="$(arg DEVICE_NAME)"/>
<param name="baudrate" value="$(arg BAUDRATE)"/>
<param name="latency_timer" value="$(arg LATENCY_TIMER)"/>
<param name="dyn_comm/retry_num"      value="3"/> <!-- 単体通信失敗時のリトライ回数，初期化にかかる時間は延びるが，メインのsub/pub周期には影響なし -->
<param name="dyn_comm/inerval_msec"   value="5"/> <!-- 単体通信失敗時のインターバル時間，初期化にかかる時間は延びるが，メインのsub/pub周期には影響なし -->
<param name="dyn_comm/varbose"        value="false"/> <!-- 通信失敗時の詳細をエラーとして出すか -->

<!-- サーボの初期設定 -->
<param name="init/auto_search_max_id"        value="45"/>   <!-- 初期化時に自動検出するサーボの最大ID，多すぎると検索に時間かかる -->
<param name="init/expected_servo_num"        value="0"/>    <!-- 初期化時に検出されたサーボがこの個数以外なら初期化失敗で止まる，0ならいくつでもok -->
<param name="init/hardware_error_auto_clean" value="false"/> <!-- 初期化時に Hardware error を自動でクリアするかどうか -->
<param name="init/torque_auto_enable"        value="true"/>  <!-- 初期化時に Torque を自動でONにするかどうか -->
<param name="term/torque_auto_disable"       value="true"/>  <!-- 終了時に Torque を自動でOFFにするかどうか -->


<!-- ループの設定 -->
<param name="loop_rate" value="250"/>
<param name="ratio/state_read"   value="1"/>    <!-- この回数に一回 State を読み取る, 0=初回のみ -->
<param name="ratio/option_read"  value="0"/>    <!-- この回数に一回 option を読み取る, 0=初回のみ -->
<param name="ratio/error_read"   value="100"/>  <!-- この回数に一回 Hardware error を読み取る, 0=初回のみ -->
<param name="ratio/varbose_loop" value="100"/>   <!-- メインループの処理時間，通信の成功率を出力, ex 100なら100回に1回出力 -->

<!-- Read/Write方式 -->
<param name="use/fast_read"  value="true"/>  <!-- Fast Sync Readを使用するかどうか． falseにすると遅い -->
<param name="use/slipt_read" value="false"/> <!-- 複数のアドレスからの読み込みを分割するか同時に行うか, trueだと遅い -->
<param name="use/slipt_write" value="true"/> <!-- 複数のアドレスへの書き込みを分割するか同時に行うか, trueでもそんなに遅くならない -->

<!-- Readする情報 -->
<param name="read/present_pwm"           value="false"/>
<param name="read/present_current"       value="true"/>
<param name="read/present_velocity"      value="true"/>
<param name="read/present_position"      value="true"/>
<param name="read/velocity_trajectory"   value="false"/>
<param name="read/position_trajectory"   value="false"/>
<param name="read/present_input_voltage" value="false"/>
<param name="read/present_temperature"   value="false"/>

<!-- デバック用 -->
<param name="max_log_width"        value="8"/>     <!-- 以下のlog出力で，サーボ何個ごとに改行を入れるか -->
<param name="varbose/callback"     value="false"/> <!-- コールバック関数の呼び出しを出力 -->
<param name="varbose/write_commad"    value="false"/> <!-- 書き込みするcommandデータのpulse値を出力 -->
<param name="varbose/write_option"    value="false"/> <!-- 書き込みするoptionデータのpulse値を出力 -->
<param name="varbose/read_state/raw"  value="false"/> <!-- 読み込んだstateデータのpulse値を出力 -->
<param name="varbose/read_state/err"  value="false"/>  <!-- stateデータの読み込みエラーを出力 -->
<param name="varbose/read_option/raw" value="false"/> <!-- 読み込んだoptionデータのpulse値を出力 -->
<param name="varbose/read_option/err" value="true"/>  <!-- optionデータの読み込みエラーを出力 -->
<param name="varbose/read_hardware_error"   value="true"/>  <!-- 検出したHardware errorを出力 -->
```

***************************

## 初期設定と注意事項

シリアル通信にはパケットの送受信の間にlatency timer分のインターバルが挟まる．
(デフォルトは16msのようであり，高速な通信の妨げとなることが多い)
安定した通信のためには，使用するUBSデバイスの latency timer とros paramの `LATENCY_TIMER` を一致させる必要がある．

ros paramの変更には，dynamixel_handler.launchの以下の部分を編集して保存する．
```xml
<!-- dynamixel_handler.launch -->
<arg name="LATENCY_TIMER" default="4"/>
```

使用するUSBデバイスのlatency timerは次のようにして変更する．
ttyUSB0 の部分は自分の環境に合わせて編集すること．
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

***************************

## Control Table との対応

### コマンド (goal values)
 - goal_pwm             : 未実装，`/dynamixel/cmd/x/pwm`をsubすると設定されるようにしたい
 - goal_current         : `/dynamixel/cmd/x/current` or `/dynamixel/cmd/x/current_position`をsubすると設定され，`loop_rate`の周期で書き込まれる．
 - goal_velocity        : `/dynamixel/cmd/x/velocity`をsubすると設定され，`loop_rate`の周期で書き込まれる．
 - goal_position        : `/dynamixel/cmd/x/position` or `/dynamixel/cmd/x/current_position` or `/dynamixel/cmd/x/extended_position` をsubすると設定され．`loop_rate`の周期で書き込まれる．
 - profile_acceleration : `/dynamixel/cmd/profile`をsubすると設定される．`loop_rate`の周期で書き込まれる．
 - profile_velocity     : `/dynamixel/cmd/profile`をsubすると設定される．`loop_rate`の周期で書き込まれる．

note: profile_{~}は制御モードが変わると勝手に0に変更されてしまう．対策済み．

### 状態 (present values)
 - present_pwm : `/dynamixel/state`として, `loop_rate`のうち，`ratio/read_state`に一回の周期でpubされる
 - present_current : `/dynamixel/state`として, `loop_rate`のうち，`ratio/read_state`に一回の周期でpubされる．
 - present_velocity    : `/dynamixel/state`として, `loop_rate`のうち，`ratio/read_state`に一回の周期でpubされる．
 - present_position      : `/dynamixel/state`として, `loop_rate`のうち，`ratio/read_state`に一回の周期でpubされる．
 - velocity_trajectory : `/dynamixel/state`として, `loop_rate`のうち，`ratio/read_state`に一回の周期でpubされる．
 - position_trajectory   : `/dynamixel/state`として, `loop_rate`のうち，`ratio/read_state`に一回の周期でpubされる．
 - present_input_voltage : `/dynamixel/state`として, `loop_rate`のうち，`ratio/read_state`に一回の周期でpubされる
 - present_temperture : `/dynamixel/state`として, `loop_rate`のうち，`ratio/read_state`に一回の周期でpubされる 

### 制限 
 - temperature_limit  : 未実装，`/dynamixel/option/limit/w`のsubで設定し，現在値を`/dynamixel/option/limit/r`としてpubできるにようにする．
 - max_voltage_limit     : 未実装，`/dynamixel/option/limit/w`のsubで設定し，現在値を`/dynamixel/option/limit/r`としてpubできるにようにする．
 - min_voltage_limit     : 未実装，`/dynamixel/option/limit/w`のsubで設定し，現在値を`/dynamixel/option/limit/r`としてpubできるにようにする．
 - pwm_limit   : 未実装，`/dynamixel/option/limit/w`のsubで設定し，現在値を`/dynamixel/option/limit/r`としてpubできるにようにする．
 - current_limit   : 未実装，`/dynamixel/option/limit/w`のsubで設定し，現在値を`/dynamixel/option/limit/r`としてpubできるにようにする．
 - acceleration_limit   : 未実装，`/dynamixel/option/limit/w`のsubで設定し，現在値を`/dynamixel/option/limit/r`としてpubできるにようにする．
 - velocity_limit      : 未実装，`/dynamixel/option/limit/w`のsubで設定し，現在値を`/dynamixel/option/limit/r`としてpubできるにようにする．
 - max_position_limit    : 未実装，`/dynamixel/option/limit/w`のsubで設定し，現在値を`/dynamixel/option/limit/r`としてpubできるにようにする．
 - min_position_limit    : 未実装，`/dynamixel/option/limit/w`のsubで設定し，現在値を`/dynamixel/option/limit/r`としてpubできるにようにする．

### ゲイン
 - velocity_i_gain       : 未実装，`/dynamixel/option/gain/w`のsubで設定し，現在値を`/dynamixel/option/gain/r`としてpubできるにようにする．
 - velocity_p_gain       : 未実装，`/dynamixel/option/gain/w`のsubで設定し，現在値を`/dynamixel/option/gain/r`としてpubできるにようにする．
 - position_d_gain       : 未実装，`/dynamixel/option/gain/w`のsubで設定し，現在値を`/dynamixel/option/gain/r`としてpubできるにようにする．
 - position_i_gain       : 未実装，`/dynamixel/option/gain/w`のsubで設定し，現在値を`/dynamixel/option/gain/r`としてpubできるにようにする．
 - position_p_gain       : 未実装，`/dynamixel/option/gain/w`のsubで設定し，現在値を`/dynamixel/option/gain/r`としてpubできるにようにする．
 - feedforward_acc_gain  : 未実装，`/dynamixel/option/gain/w`のsubで設定し，現在値を`/dynamixel/option/gain/r`としてpubできるにようにする．
 - feedforward_vel_gain  : 未実装，`/dynamixel/option/gain/w`のsubで設定し，現在値を`/dynamixel/option/gain/r`としてpubできるにようにする．

note: 制御モードによってデフォルト値が異なり，なんとモードを変えると勝手に書き換えられてしまう．制御モードをまたぐ場合の処理については検討中．

### External Ports
 - external_port_mode_{x} : 未実装，topicから制御できるようにする．
 - external_port_data_{x} : 未実装，topicから制御できるようにする．

### モード
 - operating_mode         : 対応するtopicのsubで自動で設定される．  
                            未実装，現在値を`/dynamixel/option/mode/r`としてpubされるようにする．   
                            未実装，`/dynamixel/option/mode/w`をsubして設定されるようにする．
 - drive_mode             : 未実装，現在値を`/dynamixel/option/mode/r`としてpubできるにようにする．    
                            未実装，`/dynamixel/option/mode/w`をsubして設定されるようにする．
 - torque_enable          : 接続時に自動でトルクONされる. `/dynamixel/commnad`の`command`=`'torque_on'` or `'enable'`で1,`command`=`'torque_off'` or `'disable'`で0に設定される．  
                            未実装，現在値を`/dynamixel/option/gain/r`としてpubされるようにする．   
                            未実装，`/dynamixel/option/mode/w`をsubして設定されるようにする．

### エラー
 - hardware_error_status  : `/dynamixel/error`として`loop_rate`のうち，`ratio/read_error`に一回の周期でpubされる. 

### 設定
 - return_delay_time      : not support
 - homing_offset          : ユーザーは使用不可，初期化時に0に設定され，reboot時の角度補正に用いられる．
 - moving_threshold       : not support
 - startup_configuration  : not support, buckupがあるときPIDゲインやprofile系の値を自動で復元してくれるが，PIDのデフォルト値がモードによって異なる問題があるので使わない．
 - shutdown               : not support
 - status_return_level    : not support, 常に2を前提とする
 - bus_watchbdog          : node kill時にサーボを自動停止させる機能に用いられる．   
未実装，current/velocity control時に一定時間通信切れで自動停止するようにする．

note: (bus_watchdog の設定値が1以上の時) bus_watchdogの設定値 × 20ms 通信がないと自動で動作停止処理が実行される．homing_offset が設定されている状態でこの動作停止処理が走るとなぜか homing_offsetだけ回転する．

### その他
 - led                    : not support
 - registered_instruction : not support
 - realtime_tick          : not support
 - moving                 : not support
 - moving_status          : not support

***************************

### 未実装機能
 - 精度に合わせてpubする値を丸める
 - limit系
   - paramからの設定ができるようにする
   - pubできるようにする
   - （subによる動的な設定はできなくていいか？）
 - gain系
   - paramから設定できるようにする
   - pubできるようにする
   - 電源喪失・Rebootで初期化されていしまう問題の対処
   - モード変更によってモードごとのデフォルト値に初期化される問題の対処
     - FW ver 45 以上で使えるresotre_configurationだと，バックアップ作成時点の値になってしまい，意図と異なる場合が発生しかねない． 
 - mode系
   - paramから設定できるようにする
   - pubできるようにする
 - commnad topic を service にする
 - 電流/速度制御時に通信が途切れたら自動で停止するようにする
 - External Portsをうまいことやる
 - command の設定値を callback でストアしてからメインループで write しているので，最大 1/roop_late [sec] の遅延が生じうる．
   - 現在の方法：sub callback でストアしメインループで write
     - [＋] write回数が抑えられる．
       - 各IDへの command が別の topic に乗ってきても，node 側で 1/roop_late [sec] 分の command をまとめてくれる
     - [＋] write の周期が一定以下になり，read の圧迫や負荷の変動が起きづらい
     - [－] 一度 command をストアするので，topic の sub から 最大 1/roop_late [sec] の遅延が生じてしまう．
       - 8ms未満くらいは遅れるが，そもそものtopicの遅延の方が支配的?(topic遅延が6ms，callback->writeが遅延2ms)
   - もう一つの方法：sub callback で直接 write
     - [＋] callback後の遅延は生じない
     - [－] topic の pub の仕方によってはwrite回数が増えてしまう
       - 例えば，ID:5へ指令する command topic と ID:6が別のノードからpubされているとすると，callbackは2回呼ばれる．一度ストアしてからまとめてWrite方式だとwriteは1回だが，callbackで直接Write方式だとwriteも2回

***************************

## 速度に関してメモ

### Sync Read vs Fast Sync Read("use_fast_read"パラメータ)
結論としては，読み込むデータとサーボの数が少ないならFastを使う方がよい．

Fast Sync Readは受信するパケットが全サーボ分1つながりになっている．
つまり1回の通信でやり取りするパケット数が少なくなるので，同時に読み書きするサーボが多くなると速度に違いが出てくる．
少なくとも10サーボくらいで顕著にFast Sync Readの方が早くなる．

Fast Sync Read側のデメリットとしては，直列しているサーボのどこかで断線等が起きた場合に弱いという点が挙げられる．
Fast Sync Readはパケットがつながっているため，1つでも返事をしないサーボがあるとパケットが全滅してしまう．
（これはlib_dynamixelのパケット処理の実装が悪いかもしれないが，知識不足ですぐに改善できなさそう．）
通常のSync Readはパケットが独立しているため，断線するより前のサーボからの返事は受け取ることができる．
断線や接続不良が危惧されるような状況では通信周期を犠牲にして，Sync Readを使わざるを得ないだろう．

### 複数アドレスの同時読み込み("use_split_read"パラメータ)
後述の書き込みと異なり，こちらは分割ではなく同時にするのが良い．
すなわち"use_split_read"は`false`を推奨する．

複数のアドレスからデータを読み込みたいとき，分割して読み込む場合はシリアル通信の処理時間が，アドレス数分だけ長くなる．
100Hz以上で回そうと思うと，present_current, present_velocity, present_positionという基本の3つを取り出すだけでもきつい．
自分の環境では，前述の3つくらいの同時読み込みであれば，120~180Hzくらいでる．200Hzは場合によって出るか出ないかというところ．
分割読み込みでは60~80Hzくらいで頭打ちとなってしまった．
present系の8つのアドレスすべてから読み込んでも，同時読み込みなら100Hzくらいはでる．
分割読み込みだと30Hzも怪しい．

（上記は全て，　14サーボ直列，lib_dynamixel側のLATENCY_TIMER=2ms, デバイス側のlatency timer=2msでの結果）


### 複数アドレスの同時書き込み("use_split_write"パラメータ)
書き込みに関しては，同時ではなく分割するのが良いだろう．
すなわち"use_split_write"は`true`を推奨する．

自分の環境では，"use_split_write"を`false`の状態で，12サーボに goal_current, goal_velocity, profile_acc, profile_vel, goal_position を同時にSync Writeしようとしたら，書き込みが失敗してうまく動かなかった． 
書き込むサーボが少なければ動く．
また，"use_split_write"を`true`にして，分割で書き込み，1度に書き込むアドレスを減らしても動く．
書き込みに関しては，分割して行っても処理時間はほぼ変わらない(1ms未満しか遅くならない)ので，基本は`true`としておくべき．

### その他
色々試したが，"loop_rate"=200, "ratio_read_state"=2がちょうどよさそう．
（全体周期200Hzで，そのうち2回に1回Readするので，readは100Hz）
これ以上readの周期を上げようとするとCPU使用率が高くなりすぎる．
