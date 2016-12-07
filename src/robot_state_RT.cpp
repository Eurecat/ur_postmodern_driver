/*
 * robotStateRT.cpp
 *
 * Copyright 2015 Thomas Timm Andersen
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "ur_modern_driver/robot_state_RT.h"

RobotStateRT::RobotStateRT(std::condition_variable& msg_cond):
  version_(0.0),
  time_(0.0),
  controller_timer_(0.0),
  robot_mode_(0.0),
  safety_mode_(0.0),
  speed_scaling_(0.0),
  linear_momentum_norm_(0.0),
  v_main_(0.0),
  v_robot_(0.0),
  i_robot_(0.0),
  data_published_(false),
  controller_updated_(false),
  pMsg_cond_(&msg_cond)
{
  q_target_.fill(0.0);
  qd_target_.fill(0.0);
  qdd_target_.fill(0.0);
  i_target_.fill(0.0);
  m_target_.fill(0.0);
  q_actual_.fill(0.0);
  qd_actual_.fill(0.0);
  i_actual_.fill(0.0);
  i_control_.fill(0.0);
  tool_vector_actual_.fill(0.0);
  tcp_speed_actual_.fill(0.0);
  tcp_force_.fill(0.0);
  tool_vector_target_.fill(0.0);
  tcp_speed_target_.fill(0.0);
  digital_input_bits_.fill(false);
  motor_temperatures_.fill(0.0);
  joint_modes_.fill(0.0);
  tool_accelerometer_values_.fill(0.0);
  v_actual_.fill(0.0);
}

RobotStateRT::~RobotStateRT() {
	/* Make sure nobody is waiting after this thread is destroyed */
	data_published_ = true;
	controller_updated_ = true;
	pMsg_cond_->notify_all();
}

void RobotStateRT::setDataPublished() {
	data_published_ = false;
}
bool RobotStateRT::getDataPublished() {
	return data_published_;
}

void RobotStateRT::setControllerUpdated() {
	controller_updated_ = false;
}
bool RobotStateRT::getControllerUpdated() {
	return controller_updated_;
}

void RobotStateRT::unpackDigitalInputBits(int64_t data, std::array<bool,64> &output)
{
  for (int i = 0; i < 64; i++) {
      output[i] = ((data & (1 << i)) >> i);
  }
}

double RobotStateRT::ntohd(uint64_t nf) {
	double x;
	nf = be64toh(nf);
	memcpy(&x, &nf, sizeof(x));
	return x;
}


void RobotStateRT::setVersion(double ver) {
  std::unique_lock<std::mutex> locker(val_lock_);
	version_ = ver;
}

double RobotStateRT::getVersion() {
  std::unique_lock<std::mutex> locker(val_lock_);
  return version_;
}
double RobotStateRT::getTime() {
  std::unique_lock<std::mutex> locker(val_lock_);
  return time_;
}
Vector6 RobotStateRT::getQTarget() {
  std::unique_lock<std::mutex> locker(val_lock_);
  return q_target_;
}
Vector6 RobotStateRT::getQdTarget() {
  std::unique_lock<std::mutex> locker(val_lock_);
  return qd_target_;
}
Vector6 RobotStateRT::getQddTarget() {
  std::unique_lock<std::mutex> locker(val_lock_);
  return qdd_target_;
}
Vector6 RobotStateRT::getITarget() {
  std::unique_lock<std::mutex> locker(val_lock_);
  return i_target_;
}
Vector6 RobotStateRT::getMTarget() {
  std::unique_lock<std::mutex> locker(val_lock_);
  return m_target_;
}
Vector6 RobotStateRT::getQActual() {
  std::unique_lock<std::mutex> locker(val_lock_);
  return q_actual_;
}
Vector6 RobotStateRT::getQdActual() {
  std::unique_lock<std::mutex> locker(val_lock_);
  return qd_actual_;
}
Vector6 RobotStateRT::getIActual() {
  std::unique_lock<std::mutex> locker(val_lock_);
  return i_actual_;
}
Vector6 RobotStateRT::getIControl() {
  std::unique_lock<std::mutex> locker(val_lock_);
  return i_control_;
}
Vector6 RobotStateRT::getToolVectorActual() {
  std::unique_lock<std::mutex> locker(val_lock_);
  return tool_vector_actual_;
}
Vector6 RobotStateRT::getTcpSpeedActual() {
  std::unique_lock<std::mutex> locker(val_lock_);
  return tcp_speed_actual_;
}
Vector6 RobotStateRT::getTcpForce() {
  std::unique_lock<std::mutex> locker(val_lock_);
  return tcp_force_;
}
Vector6 RobotStateRT::getToolVectorTarget() {
  std::unique_lock<std::mutex> locker(val_lock_);
  return tool_vector_target_;
}
Vector6 RobotStateRT::getTcpSpeedTarget() {
  std::unique_lock<std::mutex> locker(val_lock_);
  return tcp_speed_target_;
}
std::array<bool, 64> RobotStateRT::getDigitalInputBits() {
  std::unique_lock<std::mutex> locker(val_lock_);
  return digital_input_bits_;
}
Vector6 RobotStateRT::getMotorTemperatures() {
  std::unique_lock<std::mutex> locker(val_lock_);
  return motor_temperatures_;
}
double RobotStateRT::getControllerTimer() {
  std::unique_lock<std::mutex> locker(val_lock_);
  return controller_timer_;
}
double RobotStateRT::getRobotMode() {
  std::unique_lock<std::mutex> locker(val_lock_);
  return robot_mode_;
}
Vector6 RobotStateRT::getJointModes() {
  std::unique_lock<std::mutex> locker(val_lock_);
  return joint_modes_;
}
double RobotStateRT::getSafety_mode() {
  std::unique_lock<std::mutex> locker(val_lock_);
  return safety_mode_;
}
std::array<double,3> RobotStateRT::getToolAccelerometerValues() {
  std::unique_lock<std::mutex> locker(val_lock_);
  return tool_accelerometer_values_;
}
double RobotStateRT::getSpeedScaling() {
  std::unique_lock<std::mutex> locker(val_lock_);
  return speed_scaling_;
}
double RobotStateRT::getLinearMomentumNorm() {
  std::unique_lock<std::mutex> locker(val_lock_);
  return linear_momentum_norm_;
}
double RobotStateRT::getVMain() {
  std::unique_lock<std::mutex> locker(val_lock_);
  return v_main_;
}
double RobotStateRT::getVRobot() {
  std::unique_lock<std::mutex> locker(val_lock_);
  return v_robot_;
}
double RobotStateRT::getIRobot() {
  std::unique_lock<std::mutex> locker(val_lock_);
  return i_robot_;
}
Vector6 RobotStateRT::getVActual() {
  std::unique_lock<std::mutex> locker(val_lock_);
  return v_actual_;
}

void RobotStateRT::unpack(uint8_t * buf) {
	int64_t digital_input_bits;
	uint64_t unpack_to;
	uint16_t offset = 0;
  std::unique_lock<std::mutex> locker(val_lock_);
	int len;
	memcpy(&len, &buf[offset], sizeof(len));
	
	offset += sizeof(len);
	len = ntohl(len);
	
	//Check the correct message length is received
	bool len_good = true;
	if (version_ >= 1.6 && version_ < 1.7) { //v1.6
		if (len != 756)
			len_good = false;
	} else if (version_ >= 1.7 && version_ < 1.8) { //v1.7
		if (len != 764)
			len_good = false;
	} else if (version_ >= 1.8 && version_ < 1.9) { //v1.8
		if (len != 812)
			len_good = false;
	} else if (version_ >= 3.0 && version_ < 3.2) { //v3.0 & v3.1
		if (len != 1044)
			len_good = false;
	} else if (version_ >= 3.2 && version_ < 3.3) { //v3.2
		if (len != 1060)
			len_good = false;
	}
	
	if (!len_good) {
		printf("Wrong length of message on RT interface: %i\n", len);

		return;
	}
	
	memcpy(&unpack_to, &buf[offset], sizeof(unpack_to));
	time_ = RobotStateRT::ntohd(unpack_to);
	offset += sizeof(double);
  unpackVector(buf, offset,  q_target_);
	offset += sizeof(double) * 6;
  unpackVector(buf, offset,  qd_target_);
	offset += sizeof(double) * 6;
  unpackVector(buf, offset,  qdd_target_);
	offset += sizeof(double) * 6;
  unpackVector(buf, offset,  i_target_);
	offset += sizeof(double) * 6;
  unpackVector(buf, offset,  m_target_);
	offset += sizeof(double) * 6;
  unpackVector(buf, offset,  q_actual_);
	offset += sizeof(double) * 6;
  unpackVector(buf, offset,  qd_actual_);
	offset += sizeof(double) * 6;
  unpackVector(buf, offset,  i_actual_);
	offset += sizeof(double) * 6;
	if (version_ <= 1.9) {
		if (version_ > 1.6)
      unpackVector(buf, offset, tool_accelerometer_values_);
		offset += sizeof(double) * (3 + 15);
    unpackVector(buf, offset,  tcp_force_);
		offset += sizeof(double) * 6;
    unpackVector(buf, offset,  tool_vector_actual_);
		offset += sizeof(double) * 6;
    unpackVector(buf, offset,  tcp_speed_actual_);
	} else {
    unpackVector(buf, offset,  i_control_);
		offset += sizeof(double) * 6;
    unpackVector(buf, offset,  tool_vector_actual_);
		offset += sizeof(double) * 6;
    unpackVector(buf, offset,  tcp_speed_actual_);
		offset += sizeof(double) * 6;
    unpackVector(buf, offset,  tcp_force_);
		offset += sizeof(double) * 6;
    unpackVector(buf, offset,  tool_vector_target_);
		offset += sizeof(double) * 6;
    unpackVector(buf, offset,  tcp_speed_target_);
	}
	offset += sizeof(double) * 6;
	
	memcpy(&digital_input_bits, &buf[offset], sizeof(digital_input_bits));
  unpackDigitalInputBits(be64toh(digital_input_bits), digital_input_bits_);
	offset += sizeof(double);
  unpackVector(buf, offset,  motor_temperatures_);
	offset += sizeof(double) * 6;
	memcpy(&unpack_to, &buf[offset], sizeof(unpack_to));
	controller_timer_ = ntohd(unpack_to);
	if (version_ > 1.6) {
		offset += sizeof(double) * 2;
		memcpy(&unpack_to, &buf[offset], sizeof(unpack_to));
		robot_mode_ = ntohd(unpack_to);
		if (version_ > 1.7) {
			offset += sizeof(double);
      unpackVector(buf, offset,  joint_modes_);
		}
	}
	if (version_ > 1.8) {
		offset += sizeof(double) * 6;
		memcpy(&unpack_to, &buf[offset], sizeof(unpack_to));
		safety_mode_ = ntohd(unpack_to);
		offset += sizeof(double);
    unpackVector(buf, offset, tool_accelerometer_values_);
		offset += sizeof(double) * 3;
		memcpy(&unpack_to, &buf[offset], sizeof(unpack_to));
		speed_scaling_ = ntohd(unpack_to);
		offset += sizeof(double);
		memcpy(&unpack_to, &buf[offset], sizeof(unpack_to));
		linear_momentum_norm_ = ntohd(unpack_to);
		offset += sizeof(double);
		memcpy(&unpack_to, &buf[offset], sizeof(unpack_to));
		v_main_ = ntohd(unpack_to);
		offset += sizeof(double);
		memcpy(&unpack_to, &buf[offset], sizeof(unpack_to));
		v_robot_ = ntohd(unpack_to);
		offset += sizeof(double);
		memcpy(&unpack_to, &buf[offset], sizeof(unpack_to));
		i_robot_ = ntohd(unpack_to);
		offset += sizeof(double);
    unpackVector(buf, offset,  v_actual_);
	}

	controller_updated_ = true;
	data_published_ = true;
	pMsg_cond_->notify_all();
	
}

