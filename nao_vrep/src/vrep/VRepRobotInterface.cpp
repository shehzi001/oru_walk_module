#include "vrep/VRepJointInterface.h"


extern "C" {
    #include "extApi.h"
}
#include "v_repConst.h"

#include <iostream>
#include <string>
#include <vector>


VRepJointInterface::VRepJointInterface(const char* connection_ip, int connection_port, const std::vector<std::string> &joint_names) {

	clientID=simxStart(connection_ip,connection_port,true,true,2000,5);
	int handle = 0;
	float dummy;

	for (int i=0; i < joint_names.size(); i++) {
		const char *p = joint_names[i].c_str();

		simxGetObjectHandle(clientID, p, &handle, simx_opmode_oneshot_wait);

		//start streaming of joint positions
		simxGetJointPosition(clientID, handle, &dummy, simx_opmode_streaming);

		handles.push_back(handle);
	}

}


VRepJointInterface::~VRepJointInterface() {
	simxFinish(clientID);
}

void VRepJointInterface::setJointPosition(int index, double pos) {

	if (simxGetConnectionId(clientID) != -1) {
		//set joint to position mode
		simxSetObjectIntParameter(clientID, handles[index], 2000, 1, simx_opmode_oneshot);
		simxSetObjectIntParameter(clientID, handles[index], 2001, 1, simx_opmode_oneshot);

		simxSetJointTargetPosition(clientID, handles[index], pos, simx_opmode_oneshot);
	}
}

void VRepJointInterface::setJointPosition(int index[], std::vector<double> positions, int size) {

	for (int i=0; i < size; i++) {
		setJointPosition(index[i], positions[i]);
	}
}


void VRepJointInterface::setJointVelocity(int index, double vel) {

	if (simxGetConnectionId(clientID) != -1) {
		//set joint to velocity mode
		simxSetObjectIntParameter(clientID, handles[index], 2000, 1, simx_opmode_oneshot);
		simxSetObjectIntParameter(clientID, handles[index], 2001, 0, simx_opmode_oneshot);
		simxSetObjectIntParameter(clientID, handles[index], 1000, 0, simx_opmode_oneshot);

		simxSetJointTargetVelocity(clientID, handles[index], vel, simx_opmode_oneshot);
	}
}

void VRepJointInterface::setJointVelocity(int index[], std::vector<double> velocities, int size) {

	for (int i=0; i < size; i++) {
		setJointVelocity(index[i], velocities[i]);
	}
}

double VRepJointInterface::getJointPosition(int index) {

	float position;

	if (simxGetConnectionId(clientID) != -1) {
		simxGetJointPosition(clientID, handles[index], &position, simx_opmode_streaming);
	}

	return position;
}

void VRepJointInterface::getJointPosition(int index[], int size, std::vector<double> &positions) {

	positions.resize(size);
	for (int i=0; i < size; i++) {
		positions[i] = getJointPosition(index[i]);
	}
}


double VRepJointInterface::getJointVelocity(int index) {
    /*
    std::vector<std::string> signal_names;
    signal_names.resize(6);
    signal_names[0] = "accel-X";
    signal_names[1] = "accel-Y";
    signal_names[2] = "accel-Z";
    signal_names[3] = "gyro-X";
    signal_names[4] = "gyro-Y";
    signal_names[5] = "gyro-Z";
    signal_names[6] = "gyro-dt";
    float dummy;

    for (int i=0; i < signal_names.size(); i++) {
        const char *p = signal_names[i].c_str();
        simxGetFloatSignal(clientID, p, &dummy, simx_opmode_streaming);

        std::cout << signal_names[i] << ":" << dummy << std::endl;
    }
    */
    return 0;
}

void VRepJointInterface::getIMUData(const std::vector<std::string> &signal_names, std::vector<double> &imu_data)
{
    float dummy;

    imu_data.resize(signal_names.size());

    for (int i=0; i < signal_names.size(); i++) {
        const char *p = signal_names[i].c_str();
        simxGetFloatSignal(clientID, p, &dummy, simx_opmode_streaming);
        imu_data[i] = dummy;

        //std::cout << signal_names[i] << ":" << dummy << std::endl;
    }
}