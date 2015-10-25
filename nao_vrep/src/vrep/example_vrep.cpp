#include <iostream>
#include "oru_walk.h"
// standard headers
#include <string> // string

using namespace std;
#include "extApi.h"

#include <math.h>
#include "vrep/VRepJointInterface.h"

std::string connection_ip = "127.0.0.1";
const int connection_port = 19997;

//std::vector<string> joint_names("");

int main()
{
    // connect with V-rep
    VRepJointInterface* jointInterface = new VRepJointInterface(connection_ip, connection_port, nao_walk.joint_names);

    int index[nao_walk.joint_names.size()/2];
    std::vector<double> joint_positions;

    joint_positions.resize(nao_walk.joint_names.size()/2);

    for (int i=12; i< nao_walk.joint_names.size(); i++) {
        index[i-12] = i;
        joint_positions[i-12] = nao_walk.init_joint_angles[i];
        std::cout << joint_positions[i-12] << std::endl;
    }

    jointInterface->setJointPosition(index, joint_positions, nao_walk.joint_names.size()/2);
    sleep(2);

    for (int i=0; i< nao_walk.joint_names.size()/2; i++) {
        index[i] = i;
        joint_positions[i] = nao_walk.init_joint_angles[i];
        std::cout << joint_positions[i] << std::endl;
    }

    jointInterface->setJointPosition(index, joint_positions, nao_walk.joint_names.size()/2);
    sleep(2);
    return 0;
}
