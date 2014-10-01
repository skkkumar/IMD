
#ifndef myModule_h
#define myModule_h

#include <string>
#include <stdio.h>
#include "moveThread.h"
// #include "observeThread.h"
#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include <iostream>

#include <iCub/ctrl/math.h>

#include <highgui.h>
#include <cv.h>
// YARP_DECLARE_DEVICES(icubmod)
using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;
class myModule: public yarp::os::RFModule
{
protected:
    moveThread       *manager_thr;
//     observeThread    *obs_thr;
    RpcServer           port_rpc_human;
    Port                port_rpc;

public:
    myModule();

    virtual bool configure(ResourceFinder &rf);
    

    virtual bool interruptModule();

    virtual bool close();

    virtual bool respond(const Bottle &command);


    virtual double getPeriod();
    virtual bool   updateModule();

};
#endif
