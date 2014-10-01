
#include "moveThread.h"

moveThread::moveThread(ResourceFinder &_rf) : RateThread(20), rf(_rf) { }

bool moveThread::threadInit()
{
  setRate(int(20.0));
  
//    printf("****** problem is not here ******");
  Property optGaze("(device gazecontrollerclient)");
  optGaze.put("remote","/iKinGazeCtrl");
  optGaze.put("local","/objectDetect/gaze");
//    printf("****** problem is here ******");
  if (!clientGaze.open(optGaze)){
//     printf("Problem opening the gaze");
    return false;
  }
  // open the view
  clientGaze.view(igaze);
  
  // latch the controller context in order to preserve
  // it after closing the module
  // the context contains the tracking mode, the neck limits and so on.
  igaze->storeContext(&startup_context_id);
  
  // set trajectory time:
  igaze->setNeckTrajTime(0.8);
  igaze->setEyesTrajTime(0.4);
//        if (!obs_thr->start())
//         {
//             delete obs_thr;
//             return false;
//         }
//   port.open("/objectDetect/gaze/port");
  i = 0;
  return true;
}

void moveThread::run()
{
  Vector ang(3);

  if (i){
 // while (i){
    ang[0]= 0; 
    ang[1]= -20; 
    ang[2]= 0; 
    igaze->lookAtAbsAngles(ang); 
    igaze->waitMotionDone(2);

    ang[0]= 0; 
    ang[1]= 20; 
    ang[2]= 0; 
    igaze->lookAtAbsAngles(ang); 
    igaze->waitMotionDone(2);

    ang[0]= -20; 
    ang[1]= 0; 
    ang[2]= 0; 
    igaze->lookAtAbsAngles(ang); 
    igaze->waitMotionDone(2);

    ang[0]= 20; 
    ang[1]= 0; 
    ang[2]= 0; 
    igaze->lookAtAbsAngles(ang); 
    igaze->waitMotionDone(2);

    ang[0]= 0; 
    ang[1]= 0; 
    ang[2]= -20; 
    igaze->lookAtAbsAngles(ang); 
    igaze->waitMotionDone(2);
    ang[0]= 0; 
    ang[1]= 0; 
    ang[2]= -20;
    igaze->lookAtAbsAngles(ang); 
    igaze->waitMotionDone(2);

   // yarp::os::Time::delay(0.001);
  //}
  }
}
/*
void moveThread::getJV()
{
  Vector velVect;
  while (i){
    igaze->getJointsVelocities(velVect);
//     port.write(velVect);
  }
}*/

    void moveThread::interrupt()
    {
        mutex.wait();
//         port.interrupt();
	igaze->stopControl();
  
  // it's a good rule to restore the controller
  // context as it was before opening the module
	igaze->restoreContext(startup_context_id);
        mutex.post();
// 	obs_thr->interrupt();
    }


bool moveThread::respComd(const Bottle &Command){
  string command = Command.get(0).toString().c_str();
  if (command == "MoveHead"){
    i = 1;
//     obs_thr->respComd(Command);
//     getJV();

    return true;
    
  }
    else if (command == "StopHead"){
      i = 0;       
//       obs_thr->respComd(Command);
      return true;
    }
      
}

void moveThread::threadRelease()
{    
  // we require an immediate stop
  // before closing the client for safety reason
//   obs_thr->stop();
//   delete obs_thr;
  clientGaze.close();
  
//   port.interrupt();
//   port.close();
}
