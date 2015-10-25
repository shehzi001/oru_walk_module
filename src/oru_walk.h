/**
 * @file
 * @author Antonio Paolillo
 * @author Dimitar Dimitrov
 * @author Alexander Sherikov
 */


#ifndef ORU_WALK_H
#define ORU_WALK_H


//----------------------------------------
// INCLUDES
//----------------------------------------

// standard headers
#include <string> // string

// NAO headers
/*
#include <qi/log.hpp>

#include <alcore/alptr.h>
#include <alcore/alerror.h>

#include <alcommon/alproxy.h>
#include <alcommon/albroker.h>
#include <alcommon/almodule.h>

#include <alvalue/alvalue.h> // ALValue

#include <alproxies/dcmproxy.h>
#include <alproxies/almemoryproxy.h>

#include <almemoryfastaccess/almemoryfastaccess.h>

#include <althread/almutex.h>

#include <althread/alcriticalsection.h>

*/

// other libraries
#include <boost/bind.hpp> // callback hook
#include <boost/thread.hpp> 

// our headers
#include "WMG.h" // footsteps & parameters
#include "smpc_solver.h" // solver
#include "joints_sensors_id.h"
#include "nao_igm.h"
#include "walk_parameters.h"

#include "oru_robot_interface.h"

 #include <stdio.h>
#include <Eigen/Core>
#include <Eigen/Cholesky>

//----------------------------------------
// DEFINITIONS
//----------------------------------------

//using namespace AL;
using namespace std;

//#define ORUW_THROW(message) throw ALERROR(getName(), __FUNCTION__, message)
//#define ORUW_THROW_ERROR(message,error) throw ALERROR(getName(), __FUNCTION__, message + string (error.what()))



/**
 * @brief The main walking module class.
 */
class oru_walk //: public ALModule
{
    public:
        // constructors / destructors
        //oru_walk(ALPtr<ALBroker> broker, const string& name);
        oru_walk(const string& name, std::string connection_ip, int connection_port);
        ~oru_walk ();

        // is called automatically when a library is loaded 
        void init();
        void initRobot();

        // These methods will be advertised to other modules.
        void setStiffness(const float &);
        void walk();
        void stopWalkingRemote();
        bool dcmCallback();


        void initwalkControl();
        void walkControl();
        void postureController();

        void printMessage(const char*);

    private:
        /**
         * Copy Ctor.
         */
        oru_walk(const oru_walk &other);

        /**
         * Assignment operator
         */
        oru_walk &operator=(const oru_walk &other);

    public:
        std::vector<string> joint_names;
        std::vector<double> init_joint_angles;

    private:
        // initialization
        //void initFastRead (const vector<string>&);
        //void initFastWrite (const vector<string>&);
        void initWalkCommands ();
        void initJointAngles ();

        void MatrixPrint(unsigned int m, unsigned int n, double * A, const char * description);

        void initWalkPattern(WMG &);
        void initWalkPattern_Straight(WMG &);
        void initWalkPattern_Diagonal(WMG &);
        void initWalkPattern_Circular(WMG &);
        void initWalkPattern_Back(WMG &wmg);
        void initSolver();


        // walking
        void readSensors (jointState&);
        bool solveMPCProblem (WMG&, smpc_parameters&);
        void solveIKsendCommands (const smpc_parameters&, const smpc::state_com &, const int, WMG&);

        void correctNextSupportPosition(WMG &);
        void feedbackError (smpc::state_com &);

        void halt(const char*, const char *);
        
        // periodically called callback function


        // private variables
        //ProcessSignalConnection dcm_callback_connection;

        // Used for fast memory access
        //ALPtr<ALMemoryFastAccess> access_sensor_values; //(important)
        int* last_dcm_time_ms_ptr;

        // Used to store command to send //(important)
        //ALValue joint_commands;
        std::vector<double> joint_commands;

        nao_igm nao;
        double ref_joint_angles[LOWER_JOINTS_NUM];

        walkParameters wp;
        smpc::solver *solver;
        boost::shared_ptr<oru_robot_interface> robot_interface_;
        std::string connection_ip_;
        int connection_port_;
        int joint_indices_[JOINTS_NUM];
        std::vector<double> torso_feedback_;

        int dcm_loop_counter;
        int last_dcm_time_ms;
        bool dcm_loop_break;

        //ALPtr<DCMProxy> dcm_proxy;
        //ALPtr<ALMemoryProxy> memory_proxy;

        boost::condition_variable walk_control_condition;
        boost::mutex walk_control_mutex;
};

#endif  // ORU_WALK_H
