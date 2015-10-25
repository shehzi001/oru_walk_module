#ifndef JOINTINTERFACE_H_
#define JOINTINTERFACE_H_

#include <iostream>
#include <string>
#include <vector>

class JointInterface {
public:

	virtual void setJointPosition(int index, double pos) = 0;

	virtual void setJointPosition(int index[], std::vector<double> positions, int size) = 0;

	virtual void setJointVelocity(int index, double vel) = 0;

	virtual void setJointVelocity(int index[], std::vector<double> velocities, int size) = 0;

	virtual double getJointPosition(int index) = 0;

	virtual void getJointPosition(int index[], int size, std::vector<double> &positions) = 0;

	virtual double getJointVelocity(int index) = 0;


};

#endif /* ROBOTINTERFACE_H_ */
