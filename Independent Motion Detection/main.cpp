#include <yarp/dev/all.h>
#include <yarp/os/all.h>
#include "myModule.h"
// #include <yarp/dev/Drivers.h>
YARP_DECLARE_DEVICES(icubmod)
using namespace yarp::dev;

using namespace yarp::os;




int main(int argc, char *argv[])
{
   Network yarp;
 myModule mod;
   if (!yarp.checkNetwork())
       return -1;
   YARP_REGISTER_DEVICES(icubmod)
   ResourceFinder rf;
   rf.setVerbose(true);
   rf.setDefaultContext("objectDetection");
//    rf.setDefaultConfigFile("config.ini");
   rf.configure(argc,argv);
   rf.setDefault("name","objectDetection");
  
   return mod.runModule(rf);
}
