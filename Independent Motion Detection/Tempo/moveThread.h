
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

#define eye_horizontal_min -20
#define eye_horizontal_max 20
#define eye_vertical_min -15
#define eye_vertical_max 15
#define eye_horizontal_adjustment 10
#define eye_vertical_adjustment 5
#define eye_motion_speed_min 10
#define eye_motion_speed_max 40



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
    int state;
    int startup_context_id;
    int i;
    bool Flag;
    int jnts;
    Vector setpoints;
    Port port;
    Bottle Message;
    int eye_motion_speed;
public:
    moveThread(ResourceFinder &_rf);
    bool threadInit();
    void run();
    virtual void interrupt();
    bool respComd(const Bottle &command);
    void threadRelease();
    int generate_rand_pos(int min, int max);
    void Generate_motion(int port[]);
    void Generate_motion_constant();
    void home();
};
#endif