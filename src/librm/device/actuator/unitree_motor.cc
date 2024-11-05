
#include "unitree_motor.hpp"
#include "librm/hal/serial_interface.h"
#include "librm/hal/spi_interface.h"
#include "librm/modules/algorithm/crc32.h"

using namespace rm::device;

UnitreeMotor::UnitreeMotor(hal::SerialInterface &serial, u8 motor_id) : serial_(&serial) {
  static hal::SerialRxCallbackFunction rx_callback =
      std::bind(&UnitreeMotor::RxCallback, this, std::placeholders::_1, std::placeholders::_2);
  this->serial_->AttachRxCallback(rx_callback);

  send_data_.head.motor_id = motor_id;
}

void UnitreeMotor::SetTau(f32 tau) {
  this->ctrl_param_.tau = tau;
  this->ctrl_param_.vel = 0;
  this->ctrl_param_.pos = 0;
  this->ctrl_param_.kp = 0;
  this->ctrl_param_.kd = 0;

  SetParam(this->ctrl_param_); 
}

void UnitreeMotor::SendCommend() { this->serial_->Write(tx_buffer_, sizeof(tx_buffer_)); }

void UnitreeMotor::RxCallback(const std::vector<u8> &data, u16 rx_len) {}

void UnitreeMotor::SetParam(const ControlParam &ctrl_param) {
  send_data_.data.mode = 10;
  send_data_.data.modify_bit = 0xFF;
  send_data_.data.read_bit = 0x0;
  send_data_.data.reserved = 0x0;
  send_data_.data.Modify.L = 0;
  send_data_.data.tau = ctrl_param.tau * 256;
  send_data_.data.vel = ctrl_param.vel * 128;
  send_data_.data.pos = (f32)((ctrl_param.pos / 6.2832) * 16384.f);
  send_data_.data.kp = ctrl_param.kp * 2048;
  send_data_.data.kd = ctrl_param.kd * 1024;

  send_data_.data.LowHzMotorCmdIndex = 0;
  send_data_.data.LowHzMotorCmdByte = 0;

  send_data_.crc.ku32 = crc32_core((u32 *)(&send_data_), 7);

  std::copy(reinterpret_cast<u8 *>(&send_data_), reinterpret_cast<u8 *>(&send_data_) + sizeof(send_data_), tx_buffer_);
}