#include <iostream>
#include <yarp/dev/all.h>
#include <yarp/os/all.h>
#include "myModule.h"
YARP_DECLARE_DEVICES(icubmod)
using namespace yarp::dev;
using namespace yarp::os;
int main(int argc, char **argv[])
{
   Network yarp;
   myModule mod;
   if (!yarp.checkNetwork())
       return -1;
   ResourceFinder rf;
   rf.setVerbose(true);
   rf.setDefaultContext("objectDetection");
   rf.configure(argc,*argv);
   rf.setDefault("name","objectDetection");
   return mod.runModule(rf);
}
