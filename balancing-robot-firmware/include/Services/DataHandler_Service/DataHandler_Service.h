#pragma once
#include <Arduino.h>
#include "Config/Param_Config.h"
#include "Config/Types.h"
#include "IoHwAb/IoHwAb_HandMotion/IoHwAb_HandMotion.h"

// Khởi tạo & hiệu chuẩn “vị trí trung tính” của tay cầm
void DataHandler_Service_Init(bool forwardAxisIsX = true);
void DataHandler_Service_CalibrateNeutral(uint16_t samples = 400, uint32_t us_dt = 2000);

// Cập nhật lệnh điều khiển từ raw handmotion
// Trả về true nếu có dữ liệu mới được dùng; u_out được ghi: v_cmd_mm_s, yaw_cmd_dps, link_ok giữ từ IoHwAb_HandMotion_LinkOK()
// dt: thời gian thực tế giữa 2 lần gọi (s)
bool DataHandler_Service_Update(float dt, ControlIn *u_out);
