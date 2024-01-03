#include "dynamixel_handler.hpp"

using namespace dyn_x;

//* Main loop 内で使う全モータへの一括読み書き関数たち

/**
 * @func SyncWriteCommandValues
 * @brief 指定した範囲のコマンド値を書き込む
 * @param list_write_cmd 書き込むコマンドのEnumのリスト
*/
void DynamixelHandler::SyncWriteCommandValues(set<CmdValueIndex>& list_write_cmd){
    // 空なら即時return
    if ( list_write_cmd.empty() ) return;
    // 書き込む範囲のイテレータを取得, 分割書き込みが有効な場合書き込む範囲を1つ目のみに制限,残りは再帰的に処理する．
    auto start = min_element(list_write_cmd.begin(), list_write_cmd.end());   
    auto end = ( use_split_write_ ) ? start 
               : max_element(list_write_cmd.begin(), list_write_cmd.end());
    // 書き込みに必要な変数を用意
    vector<DynamixelAddress> target_cmd_dp_list;  // 書き込むコマンドのアドレスのベクタ
    map<uint8_t, vector<int64_t>> id_cmd_vec_map; // id と 書き込むデータのベクタのマップ
    for (size_t cmd = *start; cmd <= *end; cmd++) { // アドレスのベクタと，データのベクタの並びは対応している必要があるので，同一のループで作成する．
        const auto dp = cmd_dp_list[cmd];
        target_cmd_dp_list.push_back(dp); 
        for (auto id : id_list_) if ( series_[id]==SERIES_X ) {
            if ( !is_cmd_updated_[id] ) continue; // 更新されていない場合はスキップ
            const auto pulse  = dp.val2pulse( cmd_values_[id][cmd], model_[id]);
            id_cmd_vec_map[id].push_back( pulse );
        }
    }
    //id_cmd_vec_mapの中身を確認
    if ( varbose_write_cmd_ ) {
        char header[100]; sprintf(header, "[%d] servo(s) will be written", (int)id_cmd_vec_map.size());
        auto ss = control_table_layout(width_log_, id_cmd_vec_map, target_cmd_dp_list, string(header));
        ROS_INFO_STREAM(ss);
    }
    // SyncWriteでまとめて書き込み
    dyn_comm_.SyncWrite(target_cmd_dp_list, id_cmd_vec_map);
    // 今回書き込みした範囲を消去して残りを再帰的に処理, use_split_write_=falseの場合は全て削除されるので,再帰しない
    list_write_cmd.erase(start, ++end);
    SyncWriteCommandValues(list_write_cmd);
    // 後処理，再帰の終端で実行される
    is_cmd_updated_.clear();
}

void DynamixelHandler::SyncWriteOption_Mode(){
    return;
}

void DynamixelHandler::SyncWriteOption_Gain(){
    return;
}

void DynamixelHandler::SyncWriteOption_Limit(){
    return;
}
using std::chrono::system_clock;
using std::chrono::duration_cast;
using std::chrono::microseconds;

/**
 * @func SyncReadStateValues
 * @brief 指定した範囲の状態値を読み込む
 * @param list_read_state 読み込む状態値のEnumのリスト
 * @return 読み取りの成功率, なんで再帰で頑張って実装してるんだろう．．．
*/
double DynamixelHandler::SyncReadStateValues(set<StValueIndex> list_read_state){
    // 空なら即時return
    if ( list_read_state.empty() ) return 1.0;
    // 読み込む範囲のstate_dp_listのインデックスを取得
    auto start = min_element(list_read_state.begin(), list_read_state.end());
    auto end = ( use_split_read_ ) ? start
               : max_element(list_read_state.begin(), list_read_state.end());
    // 読み込みに必要な変数を用意
    vector<DynamixelAddress> target_state_dp_list;
    for (size_t st=*start; st<=*end; st++) target_state_dp_list.push_back(state_dp_list[st]);
    vector<uint8_t> target_id_list;
    for (auto id : id_list_) if ( series_[id]==SERIES_X ) target_id_list.push_back(id);
    // SyncReadでまとめて読み込み
    const auto id_st_vec_map = ( use_fast_read_ ) // fast read を使うかどうか．　途中で切り替えるとtimeout後に来るデータによってSyncReadが何度も失敗するので注意
        ? dyn_comm_.SyncRead_fast(target_state_dp_list, target_id_list)
        : dyn_comm_.SyncRead     (target_state_dp_list, target_id_list);
    const size_t N_total = target_id_list.size();
    const size_t N_suc   = id_st_vec_map.size();
    const bool is_timeout_  = dyn_comm_.timeout_last_read();
    const bool is_comm_err_ = dyn_comm_.comm_error_last_read();
    has_hardware_err_ = dyn_comm_.hardware_error_last_read();
    // 通信エラーの表示
    if ( varbose_read_st_err_ ) if ( is_timeout_ || is_comm_err_ ) {
        vector<uint8_t> failed_id_list;
        for ( auto id : target_id_list ) if ( id_st_vec_map.find(id) == id_st_vec_map.end() ) failed_id_list.push_back(id);
        char header[99]; sprintf(header, "[%d] servo(s) failed to read", (int)(N_total - N_suc));
        auto ss = id_list_layout(failed_id_list, string(header)+( is_timeout_ ? " (time out)" : " (some kind packet error)"));
        ROS_WARN_STREAM(ss);
    }
    // id_st_vec_mapの中身を確認
    if ( varbose_read_st_ ) if ( N_suc>0 ) {
        char header[99]; sprintf(header, "[%d] servo(s) are read", (int)N_suc);
        auto ss = control_table_layout(width_log_, id_st_vec_map, target_state_dp_list, string(header));
        ROS_INFO_STREAM(ss);
        if ( has_hardware_err_ ) ROS_WARN("Hardware Error are detected");
    }
    // state_values_に反映
    const int num_state = *end-*start+1;
    for (int i = 0; i < num_state; i++) {
        const auto dp = target_state_dp_list[i];
        for (auto pair : id_st_vec_map) {
            const uint8_t id = pair.first;
            const int64_t data_int = pair.second[i];
            state_values_[id][*start+i] = dp.pulse2val( data_int, model_[id]);
        }
    }
    // 今回読み込んだ範囲を消去して残りを再帰的に処理, use_split_read_=falseの場合は全て削除されるので,再帰しない
    list_read_state.erase(start, ++end);
    double suc_rate_prev = SyncReadStateValues(list_read_state)*list_read_state.size() / (num_state+list_read_state.size());
    return suc_rate_prev + N_suc/(double)N_total*num_state/(num_state+list_read_state.size());
}

/**
 * @func SyncReadHardwareError
 * @brief ハードウェアエラーを読み込む
 * @return 読み取りの成功率
*/
double DynamixelHandler::SyncReadHardwareErrors(){
    if ( !has_hardware_err_ ) { hardware_error_.clear(); return 1.0; } // 事前にエラーが検出できていない場合は省略

    vector<uint8_t> target_id_list;
    for (int id : id_list_) if ( series_[id]==SERIES_X ) target_id_list.push_back(id);
    
    auto id_error_map =  ( use_fast_read_ ) 
        ? dyn_comm_.SyncRead_fast(hardware_error_status, target_id_list)
        : dyn_comm_.SyncRead     (hardware_error_status, target_id_list);

    if ( dyn_comm_.timeout_last_read() ) return 0.0; // 読み込み失敗

    //  hardware_error_に反映
    for ( auto pair : id_error_map ){
        const uint8_t id = pair.first;
        const uint8_t error = pair.second;
        if ((error >> HARDWARE_ERROR_INPUT_VOLTAGE)     & 0b1 ) hardware_error_[id][INPUT_VOLTAGE     ] = true;
        if ((error >> HARDWARE_ERROR_MOTOR_HALL_SENSOR) & 0b1 ) hardware_error_[id][MOTOR_HALL_SENSOR ] = true;
        if ((error >> HARDWARE_ERROR_OVERHEATING)       & 0b1 ) hardware_error_[id][OVERHEATING       ] = true;
        if ((error >> HARDWARE_ERROR_MOTOR_ENCODER)     & 0b1 ) hardware_error_[id][MOTOR_ENCODER     ] = true;
        if ((error >> HARDWARE_ERROR_ELECTRONICAL_SHOCK)& 0b1 ) hardware_error_[id][ELECTRONICAL_SHOCK] = true;
        if ((error >> HARDWARE_ERROR_OVERLOAD)          & 0b1 ) hardware_error_[id][OVERLOAD          ] = true;
    }

    // コンソールへの表示
    if ( varbose_read_hwerr_ ) {
        ROS_WARN("Hardware error are Checked");
        for (auto id : target_id_list) {
            if (hardware_error_[id][INPUT_VOLTAGE     ]) ROS_ERROR(" * servo id [%d] has INPUT_VOLTAGE error",      id);
            if (hardware_error_[id][MOTOR_HALL_SENSOR ]) ROS_ERROR(" * servo id [%d] has MOTOR_HALL_SENSOR error",  id);
            if (hardware_error_[id][OVERHEATING       ]) ROS_ERROR(" * servo id [%d] has OVERHEATING error",        id);
            if (hardware_error_[id][MOTOR_ENCODER     ]) ROS_ERROR(" * servo id [%d] has MOTOR_ENCODER error",      id);
            if (hardware_error_[id][ELECTRONICAL_SHOCK]) ROS_ERROR(" * servo id [%d] has ELECTRONICAL_SHOCK error", id);
            if (hardware_error_[id][OVERLOAD          ]) ROS_ERROR(" * servo id [%d] has OVERLOAD error",           id);
        }
    }
    return id_error_map.size()/double(target_id_list.size());
}

double DynamixelHandler::SyncReadOption_Mode(){
    return 1.0;
}

double DynamixelHandler::SyncReadOption_Gain(){
    return 1.0;
}

double DynamixelHandler::SyncReadOption_Limit(){
    vector<uint8_t> target_id_list;
    for (int id : id_list_) if ( series_[id]==SERIES_X ) target_id_list.push_back(id);

    auto id_limit_vec_map = ( use_fast_read_ )
        ? dyn_comm_.SyncRead_fast(opt_limit_dp_list, target_id_list)
        : dyn_comm_.SyncRead     (opt_limit_dp_list, target_id_list);
    const bool is_timeout   = dyn_comm_.timeout_last_read();
    const bool has_comm_err = dyn_comm_.comm_error_last_read();
    // 通信エラーの表示
    if ( varbose_read_opt_err_ ) if ( has_comm_err || is_timeout ) {
        vector<uint8_t> failed_id_list;
        for ( auto id : target_id_list ) if ( id_limit_vec_map.find(id) == id_limit_vec_map.end() ) failed_id_list.push_back(id);
        char header[100]; sprintf(header, "[%d] servo(s) failed to read", (int)(target_id_list.size() - id_limit_vec_map.size()));
        auto ss = id_list_layout(failed_id_list, string(header) + (is_timeout? " (time out)" : " (some kind packet error)"));
        ROS_WARN_STREAM(ss);
    }
    // ACCELERATION_LIMITに関してだけ修正を入れる．0はほぼあり得ないかつ0の時profile_accの設定ができないので，適当に大きな値に変更する．
    vector<uint8_t> fixed_id_list;
    for (auto pair : id_limit_vec_map) {
        const uint8_t id = pair.first;
        const int64_t acc = pair.second[ACCELERATION_LIMIT];
        if ( acc != 0 ) continue;
        fixed_id_list.push_back(id);
        id_limit_vec_map[id][ACCELERATION_LIMIT] = 32767;
    }
    // id_limit_vec_mapの中身を確認
    if ( varbose_read_opt_ ) if ( id_limit_vec_map.size()>0 ) {
        char header[100]; sprintf(header, "[%d] servo(s) are read", (int)id_limit_vec_map.size());
        auto ss = control_table_layout(width_log_, id_limit_vec_map, opt_limit_dp_list, string(header));
        ROS_INFO_STREAM(ss);
        if ( !fixed_id_list.empty() ) {
            char header[100]; sprintf(header,"\n[%d] servo(s)' accelerarion_limit is 0, change to 32767", (int)fixed_id_list.size());
            auto ss = id_list_layout(fixed_id_list, string(header));
            ROS_WARN_STREAM(ss);
        }
    }
    // option_limit_に反映
    for ( auto opt_lim=0; opt_lim<opt_limit_dp_list.size(); opt_lim++) {
        DynamixelAddress dp = opt_limit_dp_list[opt_lim];
        for (auto pair : id_limit_vec_map) {
            const uint8_t id = pair.first;
            const int64_t data_int = pair.second[opt_lim];
            option_limit_[id][opt_lim] = dp.pulse2val( data_int, model_[id] );
        }
    }
    return id_limit_vec_map.size() / (double)target_id_list.size();
}