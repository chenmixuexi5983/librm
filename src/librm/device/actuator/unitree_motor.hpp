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

#include "librm/hal/serial.h"
#include "librm/core/typedefs.h"
#include "librm/modules/algorithm/crc32.h"
#include "librm/modules/algorithm/crc_ccitt.h"

#include <string>

union com_data32 {
  i32 L;
  u8 ku8[4];
  u16 ku16[2];
  u32 ku32;
  f32 F;
};

struct control_param {
  f32 tau;
  f32 vel;
  f32 pos;
  f32 kp;
  f32 kd;
};

struct feedback_param {
  u8 mode;
  i8 temp;
  u8 m_error;

  f32 tau;
  f32 vel;

  i16 acc;
  f32 pos;

  f32 gyro[3];
  f32 accel[3];
};

struct com_head {
  u8 head[2]{0xFE, 0xEE};
  u8 motor_id;
  u8 reserved;
};

struct com_data_send {
  u8 mode;
  u8 modify_bit;
  u8 read_bit;
  u8 reserved;

  com_data32 Modify;

  i16 tau;
  i16 vel;
  i32 pos;
  i16 kp;
  i16 kd;

  u8 LowHzMotorCmdIndex;
  u8 LowHzMotorCmdByte;

  com_data32 Res[1];
} __attribute__((packed));

struct com_data_recv {
  u8 mode;
  u8 read_bit;
  i8 temp;
  u8 m_error;

  com_data32 read;
  i16 tau;

  i16 vel;
  f32 low_vel;

  i16 vel_ref;
  f32 low_vel_ref;

  i16 acc;
  i16 out_acc;

  i32 pos;
  i32 out_pos;

  i16 gyro[3];
  i16 accel[3];

  i16 f_gyro[3];
  i16 f_acc[3];
  i16 f_mag[3];
  u8 f_temp;

  i16 force16;
  i8 force8;

  u8 f_error;

  i8 res[1];
} __attribute__((packed));

struct send_data {
  com_head head;
  com_data_send data;
  com_data32 crc;
} __attribute__((packed));

struct recv_data {
  com_head head;
  com_data_recv data;
  com_data32 crc;
} __attribute__((packed));

namespace rm::device {
class UnitreeMotor {
 public:
  UnitreeMotor(hal::SerialInterface &serial, u8 motor_id = 0x0);
  ~UnitreeMotor() = default;

  void SetTau(f32 tau);

  void SendCommend();

  void RxCallback(const std::vector<u8> &data, u16 rx_len);

  [[nodiscard]] f32 GetTau() { return this->fb_param_.tau; }
  [[nodiscard]] f32 GetVel() { return this->fb_param_.vel; }
  [[nodiscard]] i16 GetAcc() { return this->fb_param_.acc; }
  [[nodiscard]] f32 GetPos() { return this->fb_param_.pos; }

 private:
  void SetParam(const control_param& ctrl_param);

 private:
  hal::SerialInterface *serial_;

  send_data send_data_;
  recv_data recv_data_;
  control_param ctrl_param_;
  feedback_param fb_param_;
  u8 tx_buffer_[34]{0};

 public:
  u8 test_len;
};

}  // namespace rm::device

#endif  // LIBRM_DEVICE_ACTUATOR_UNITREE_MOTOR_HPP