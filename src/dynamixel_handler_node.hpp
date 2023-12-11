// 今度ちゃんとまとめる

// #ifndef DYNAMIXEL_HANDLER_H_
// #define DYNAMIXEL_HANDLER_H_

// #include <vector>
// #include <cmath>
// #include <string>

// #include "download/port_handler.h"
// #include "dynamixel_parameters.h"

// class DynamixelHandler {
//     public:
//     DynamixelHandler() {
//         status_return_level_ = 2;
//     }


//     private:
//     uint16_t CalcChecksum(uint8_t data[], uint8_t length);
//     void EncodeDataWrite(DynamixelDataType type, int64_t data_int);
//     int64_t DecodeDataRead(DynamixelDataType type);

//     const char *port_name_;
//     PortHandler *port_handler_;
//     uint32_t baudrate_;
//     uint8_t status_return_level_;
//     double time_per_byte_;
//     uint8_t  data_write_[4];
//     uint8_t  data_read_[4];
//     bool error_last_read_;
// };


// #endif /* DYNAMIXEL_HANDLER_H_ */
