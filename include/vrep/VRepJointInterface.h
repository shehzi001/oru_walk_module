/*
 * YouBot.h
 *
 *  Created on: Oct 17, 2013
 *      Author: matthias
 */

#ifndef VREPJOINTINTERFACE_H_
#define VREPJOINTINTERFACE_H_

#include "JointInterface.h"

#include <iostream>
#include <string>
#include <vector>

class VRepJointInterface: public JointInterface {
private:

	int clientID;
	std::vector<int> handles;

public:
	VRepJointInterface(const char* connection_ip, int connection_port, const std::vector<std::string> &joint_names);

	virtual ~VRepJointInterface();

	void setJointPosition(int index, double pos);

	void setJointPosition(int index[], std::vector<double> positions, int size);

	void setJointVelocity(int index, double vel);

	void setJointVelocity(int index[], std::vector<double> velocities, int size);

	double getJointPosition(int index);

	void getJointPosition(int index[], int size, std::vector<double> &positions);

	double getJointVelocity(int index);

	void getIMUData(const std::vector<std::string> &signal_names, std::vector<double> &imu_data);
};

#endif /* VREPJOINTINTERFACE_H_ */
