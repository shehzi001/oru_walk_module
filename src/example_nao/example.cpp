#include <iostream>
#include "oru_walk.h"
// standard headers
#include <string> // string
#include <time.h>

using namespace std;
#include "extApi.h"

#include <math.h>
#include "vrep/VRepJointInterface.h"

std::string connection_ip = "127.0.0.1";
const int connection_port = 19997;

void sleepcp(int milliseconds) // cross-platform sleep function
{
    clock_t time_end;
    time_end = clock() + milliseconds * CLOCKS_PER_SEC/1000;
    while (clock() < time_end)
    {
    }
}

int main()
{
    const string module_name = "nao walk";
    oru_walk nao_walk(module_name, connection_ip, connection_port);
    nao_walk.walk();
    sleep(5.0);
    //nao_walk.walkControl();

    nao_walk.printMessage("Walking module is initilized");
    for (;;)
    {
        //usleep(100000);
        sleepcp(100);
        if(nao_walk.dcmCallback()) break;
    }

    nao_walk.stopWalkingRemote();

    nao_walk.printMessage("Walking module is finished");
    sleep(1.0);

    return 0;
}
