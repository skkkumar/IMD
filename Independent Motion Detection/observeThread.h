
#ifndef observeThread_h
#define observeThread_h

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

// YARP_DECLARE_DEVICES(icubmod)

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;


class observeThread: public RateThread
{
protected:
    ResourceFinder &rf;
Semaphore                           mutex;
    PolyDriver      clientobsGaze;
    IGazeControl   *ogaze;
    int i;
    int startup_context_id;
    Port port;
      Vector velVect;
//     RpcServer port;

public:
    observeThread(ResourceFinder &_rf);
    bool threadInit();

    void run();
    bool respComd(const Bottle &Command);
    virtual void interrupt();
    
    void threadRelease();
};

#endif
