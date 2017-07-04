/*
 * ur_driver
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

#ifndef UR_DRIVER_H_
#define UR_DRIVER_H_

#include <math.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <condition_variable>
#include <mutex>
#include <string>
#include <vector>
#include "do_output.h"
#include "ur_communication.h"
#include "ur_realtime_communication.h"

#include <chrono>

class UrDriver {
  private:
	double maximum_time_step_;
	double minimum_payload_;
	double maximum_payload_;
	std::vector<std::string> joint_names_;
	std::string ip_addr_;
	const int MULT_JOINTSTATE_ = 1000000;
	const int MULT_TIME_ = 1000000;
	const unsigned int REVERSE_PORT_;
	int incoming_sockfd_;
	int new_sockfd_;
	bool reverse_connected_;
	double servoj_time_;
	bool executing_traj_;
	double firmware_version_;
	double servoj_lookahead_time_;
	double servoj_gain_;

  public:
	UrRealtimeCommunication* rt_interface_;
	UrCommunication* sec_interface_;

	UrDriver(std::condition_variable& rt_msg_cond, std::condition_variable& msg_cond,
	         std::string host, unsigned int reverse_port = 50007,
	         double servoj_time = 0.016, unsigned int safety_count_max = 12,
	         double max_time_step = 0.08, double min_payload = 0.,
	         double max_payload = 1., double servoj_lookahead_time = 0.03,
	         double servoj_gain = 300.);
	bool start();
	void halt();

	void setSpeed(const Vector6& velocities, double acc = 100.);

	bool doTraj(const std::vector<double>& inp_timestamps,
	            const std::vector<Vector6>& inp_positions,
	            const std::vector<Vector6>& inp_velocities);
	void servoj(const Vector6& positions, int keepalive = 1);

	void stopTraj();

	bool uploadProg();
	bool openServo();
	void closeServo(const Vector6& positions);
	void closeServo();

	Vector6 interp_cubic(double t, double T, const Vector6& p0_pos, const Vector6& p1_pos,
	                     const Vector6& p0_vel, const Vector6& p1_vel);

	const std::vector<std::string>& getJointNames();
	void setJointNames(const std::vector<std::string>& jn);
	void setToolVoltage(unsigned int v);
	void setFlag(unsigned int n, bool b);
	void setDigitalOut(unsigned int n, bool b);
	void setAnalogOut(unsigned int n, double f);
	bool setPayload(double m);

	void setMinPayload(double m);
	void setMaxPayload(double m);
	void setServojTime(double t);
	void setServojLookahead(double t);
	void setServojGain(double g);
};

#endif /* UR_DRIVER_H_ */
