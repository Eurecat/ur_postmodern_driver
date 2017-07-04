/*
 * robotStateRT.h
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

#ifndef ROBOT_STATE_RT_H_
#define ROBOT_STATE_RT_H_

#include <inttypes.h>
#include <netinet/in.h>
#include <stdlib.h>
#include <string.h>
#include <condition_variable>
#include <mutex>
#include <vector>
#include "robot_state.h"
#include "ur_driver_common.h"

class RobotStateRT {
  private:
	double version_;  // protocol version

	double time_;                 // Time elapsed since the controller was started
	Vector6 q_target_;            // Target joint positions
	Vector6 qd_target_;           // Target joint velocities
	Vector6 qdd_target_;          // Target joint accelerations
	Vector6 i_target_;            // Target joint currents
	Vector6 m_target_;            // Target joint moments (torques)
	Vector6 q_actual_;            // Actual joint positions
	Vector6 qd_actual_;           // Actual joint velocities
	Vector6 i_actual_;            // Actual joint currents
	Vector6 i_control_;           // Joint control currents
	Vector6 tool_vector_actual_;  // Actual Cartesian coordinates of the tool:
	                              // (x,y,z,rx,ry,rz), where rx, ry and rz is
	                              // a
	// rotation vector representation of the tool orientation
	Vector6 tcp_speed_actual_;  // Actual speed of the tool given in Cartesian coordinates
	Vector6 tcp_force_;         // Generalised forces in the TC
	Vector6 tool_vector_target_;  // Target Cartesian coordinates of the tool:
	                              // (x,y,z,rx,ry,rz), where rx, ry and rz is
	                              // a
	// rotation vector representation of the tool orientation
	Vector6 tcp_speed_target_;  // Target speed of the tool given in Cartesian coordinates
	std::array<bool, 64> digital_input_bits_;  // Current state of the digital inputs.
	                                           // NOTE: these are bits encoded as
	// int64_t, e.g. a value of 5 corresponds to bit 0 and bit 2 set high
	Vector6 motor_temperatures_;  // Temperature of each joint in degrees celsius
	double controller_timer_;     // Controller realtime thread execution time
	double robot_mode_;           // Robot mode
	Vector6 joint_modes_;         // Joint control modes
	double safety_mode_;          // Safety mode
	std::array<double, 3> tool_accelerometer_values_;  // Tool x,y and z accelerometer
	                                                   // values (software version 1.7)
	double speed_scaling_;         // Speed scaling of the trajectory limiter
	double linear_momentum_norm_;  // Norm of Cartesian linear momentum
	double v_main_;                // Masterboard: Main voltage
	double v_robot_;               // Matorborad: Robot voltage (48V)
	double i_robot_;               // Masterboard: Robot current
	Vector6 v_actual_;             // Actual joint voltages

	std::mutex val_lock_;  // Locks the variables while unpack parses data;

	std::condition_variable* pMsg_cond_;  // Signals that new vars are available
	bool data_published_;                 // to avoid spurious wakes
	bool controller_updated_;             // to avoid spurious wakes

	std::array<bool, 64> unpackDigitalInputBits(int64_t data);

  public:
	RobotStateRT(std::condition_variable& msg_cond);
	~RobotStateRT();
	double getVersion();
	double getTime();
	Vector6 getQTarget();
	Vector6 getQdTarget();
	Vector6 getQddTarget();
	Vector6 getITarget();
	Vector6 getMTarget();
	Vector6 getQActual();
	Vector6 getQdActual();
	Vector6 getIActual();
	Vector6 getIControl();
	Vector6 getToolVectorActual();
	Vector6 getTcpSpeedActual();
	Vector6 getTcpForce();
	Vector6 getToolVectorTarget();
	Vector6 getTcpSpeedTarget();
	std::array<bool, 64> getDigitalInputBits();
	Vector6 getMotorTemperatures();
	double getControllerTimer();
	double getRobotMode();
	Vector6 getJointModes();
	double getSafety_mode();
	std::array<double, 3> getToolAccelerometerValues();
	double getSpeedScaling();
	double getLinearMomentumNorm();
	double getVMain();
	double getVRobot();
	double getIRobot();

	void setVersion(double ver);

	void setDataPublished();
	bool getDataPublished();
	bool getControllerUpdated();
	void setControllerUpdated();
	Vector6 getVActual();
	void unpack(uint8_t* buf);
};

#endif /* ROBOT_STATE_RT_H_ */
