
#ifndef moveThread_h
#define moveThread_h
#include <iostream>
#include <string>
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/Semaphore.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/GazeControl.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/all.h>
#include "observeThread.h"
// YARP_DECLARE_DEVICES(icubmod)

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;


class moveThread: public RateThread
{
protected:
    ResourceFinder &rf;
Semaphore                           mutex;
Property optGaze;
    PolyDriver      robotHead;
    IPositionControl *pos1;
    IVelocityControl *vel1;
    IEncoders *enc1;
//     observeThread       *obs_thr;
    int state;
    int startup_context_id;
    int i;
        bool Flag;
	    int jnts;
// 	yarp::sig::Vector setpoints;
//     int jnts = 0;
    Vector setpoints;
    Port port;
      Bottle Message;
//     RpcServer port;

public:
    moveThread(ResourceFinder &_rf);
    bool threadInit();

    void run();
    
    virtual void interrupt();


    
    bool respComd(const Bottle &command);

    void threadRelease();
//     void getJV();
};
#endif