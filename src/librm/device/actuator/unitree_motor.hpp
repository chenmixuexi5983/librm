/*
  Copyright (c) 2024 XDU-IRobot

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/

/**
 * @file  librm/device/actuator/unitree_motor.hpp
 * @brief 宇树电机类库
 */

#ifndef LIBRM_DEVICE_ACTUATOR_UNITREE_MOTOR_HPP
#define LIBRM_DEVICE_ACTUATOR_UNITREE_MOTOR_HPP

#include "librm/hal/linux/serial.h"
#include "librm/hal/serial.h"
#include "librm/core/typedefs.h"
#include "librm/modules/algorithm/crc32.h"
#include "librm/modules/algorithm/crc_ccitt.h"

#include <string>

union ComData32 {
  i32 L;
  u8 ku8[4];
  u16 ku16[2];
  u32 ku32;
  f32 F;
};

struct ControlParam {
  f32 tau;
  f32 vel;
  f32 pos;
  f32 kp;
  f32 kd;
};

struct ComHead {
  u8 head[2]{0xFE, 0xEE};
  u8 motor_id;
  u8 reserved;
};

struct ComData {
  u8 mode;
  u8 modify_bit;
  u8 read_bit;
  u8 reserved;

  ComData32 Modify;

  i16 tau;
  i16 vel;
  i32 pos;
  i16 kp;
  i16 kd;

  u8 LowHzMotorCmdIndex;
  u8 LowHzMotorCmdByte;

  ComData32 Res[1];
} __attribute__((packed));

struct send_data {
  ComHead head;
  ComData data;
  ComData32 crc;
} __attribute__((packed));

namespace rm::device {
class UnitreeMotor {
 public:
  UnitreeMotor(u8 motor_id = 0x0);
  ~UnitreeMotor() = default;

  void SetTau(f32 tau);

 public:
  u8 tx_buffer_[34]{0};

 private:
  void SetParam(ControlParam ctrl_param);

 private:
  std::string dev_{};

  send_data send_data_;
  ControlParam ctrl_param_;
};

}  // namespace rm::device

#endif  // LIBRM_DEVICE_ACTUATOR_UNITREE_MOTOR_HPP