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
 * @file  librm/modules/algorithm/chassis_fkik.h
 * @brief 各种常见底盘构型的正逆运动学解算
 */

#ifndef LIBRM_MODULES_ALGORITHM_CHASSIS_FKIK_H
#define LIBRM_MODULES_ALGORITHM_CHASSIS_FKIK_H

#include "librm/core/typedefs.h"

namespace rm::modules::algorithm {

/**
 * @brief 麦轮底盘
 */
class MecanumChassis {
 public:
  MecanumChassis() = delete;
  ~MecanumChassis() = default;
  MecanumChassis(f32 wheel_base, f32 wheel_track);

  auto Forward(f32 vx, f32 vy, f32 wz);
  // TODO: ik
  inline auto forward_result() const { return forward_result_; }

 private:
  struct {
    f32 lf_speed, rf_speed, lr_speed, rr_speed;
  } forward_result_{};
  f32 wheel_base_;   // 轮子间距
  f32 wheel_track_;  // 轮子轴距
};

/**
 * @brief 四舵轮底盘
 */
class SteeringChassis {
 public:
  explicit SteeringChassis(f32 chassis_radius);
  auto Forward(f32 vx, f32 vy, f32 w);
  auto Forward(f32 vx, f32 vy, f32 w, f32 current_lf_angle, f32 current_rf_angle, f32 current_lr_angle,
               f32 current_rr_angle);
  inline auto forward_result() const { return forward_result_; }

 private:
  struct {
    double lf_steer_position, rf_steer_position, lr_steer_position, rr_steer_position;
    double lf_wheel_speed, rf_wheel_speed, lr_wheel_speed, rr_wheel_speed;
  } forward_result_{};
  double chassis_radius_;
};

/**
 * @brief 四全向轮底盘
 */
class QuadOmniChassis {
 public:
  QuadOmniChassis() = default;
  ~QuadOmniChassis() = default;

  auto Forward(f32 vx, f32 vy, f32 wz);
  auto Inverse(f32 front_wheel_speed, f32 back_wheel_speed, f32 left_wheel_speed, f32 right_wheel_speed);
  inline auto forward_result() const { return forward_result_; }
  inline auto inverse_result() const { return inverse_result_; }

  struct {
    f32 vx, vy, wz;
  } inverse_result_{};
  struct {
    f32 front_wheel_speed, back_wheel_speed, left_wheel_speed, right_wheel_speed;
  } forward_result_{};
};

}  // namespace rm::modules::algorithm

#endif  // LIBRM_MODULES_ALGORITHM_CHASSIS_FKIK_H
