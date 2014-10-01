
#include "moveThread.h"

moveThread::moveThread(ResourceFinder &_rf) : RateThread(20), rf(_rf) { }

bool moveThread::threadInit()
{
  setRate(int(20.0));
  
  port.open("/moveHead/check:o");
  
   cout<<"****** problem is not here ******"<<endl<<flush;
    optGaze.put("device", "remote_controlboard");
    optGaze.put("local", "/mover/motor/client");
    optGaze.put("remote", "/icubSim/head");
  
    robotHead.open(optGaze);

    if (!robotHead.isValid()) 
    {
        printf("Cannot connect to robot head\n");
        return 1;
    }
  
      robotHead.view(pos1);
    robotHead.view(vel1);
    robotHead.view(enc1);
  
    if (pos1==NULL || vel1==NULL || enc1==NULL) 
    {
        printf("Cannot get interface to robot head\n");
        robotHead.close();
        return 1;
    }
//     pos1->getAxes(&jnts);
//     setpoints.resize(jnts);


    jnts = 0;
    pos1->getAxes(&jnts);
    setpoints.resize(jnts);
    Flag = false;
    
//   for (i = 0; i < jnts; i++) {
//   setpoints[i] = 10.0;
//   }
//   pos1->setRefAccelerations(setpoints.data());
 
  for (i = 0; i < jnts; i++) {
  setpoints[i] = 10.0;
  pos1->setRefSpeed(i, setpoints[i]);
  }
    
    
            for (int i=0; i<jnts; i++) 
            {
                setpoints[i] = 0;
            }
//             cout<<"Problem Not here 1"<<endl<<flush;
	    pos1->positionMove(setpoints.data());
// 	    cout<<"Problem Not here 2"<<endl<<flush;
	    pos1->checkMotionDone(&Flag);
// 	    cout<<"Problem Not here 3"<<endl<<flush;
	    while (!Flag){
// 	      cout<<"Problem Not here 4"<<endl<<flush;
	    pos1->checkMotionDone(&Flag);}
  i = 0;
  return true;
}

void moveThread::run()
{
//   Vector ang(3);
//             for (int i=0; i<jnts; i++) 
//             {
//                 setpoints[i] = 0;
//             }
//   cout<<"Command not received"<<endl<<flush;
  if (1){
//       cout<<"=====Command received====="<<endl<<flush;
//     setpoints[4] = 40;
//     vel1->velocityMove(setpoints.data());
//     pos1.
//     pos1->checkMotionDone()
//     while()
//             for (int i=0; i<jnts; i++) 
//             {
//                 setpoints[i] = 0;
//             }
// //             cout<<"Problem Not here 1"<<endl<<flush;
// 	    pos1->positionMove(setpoints.data());
// // 	    cout<<"Problem Not here 2"<<endl<<flush;
// 	    pos1->checkMotionDone(&Flag);
// // 	    cout<<"Problem Not here 3"<<endl<<flush;
// 	    while (!Flag){
// // 	      cout<<"Problem Not here 4"<<endl<<flush;
// 	    pos1->checkMotionDone(&Flag);}
// 	    cout<<"Problem Not here 5"<<endl<<flush;
//             if (conf>0.5) 
//             {
// 	      setpoints[1]=-45;
//                 setpoints[3] = 0;
                setpoints[4] = -20;
//             } 
//             else 
//             {
//                 setpoints[3] = 0;
//                 setpoints[4] = 0;
//             }
//             vel1->velocityMove(setpoints.data());
		
// 		cout<<"Problem Not here 6"<<endl<<flush;
	    pos1->positionMove(setpoints.data());
	    pos1->checkMotionDone(&Flag);
	    while (!Flag){
	    pos1->checkMotionDone(&Flag);}
	    
	    
	    setpoints[4] = 20;
	    pos1->positionMove(setpoints.data());
	    
	    pos1->checkMotionDone(&Flag);
	    while (!Flag){
	    pos1->checkMotionDone(&Flag);}
  }

  Message.addInt(1);
        port.write(Message);
}


    void moveThread::interrupt()
    {
        mutex.wait();
        port.interrupt();

  // it's a good rule to restore the controller
  // context as it was before opening the module
// 	igaze->restoreContext(startup_context_id);
        mutex.post();
// 	obs_thr->interrupt();
    }


bool moveThread::respComd(const Bottle &Command){
  string command = Command.get(0).toString().c_str();
//   cout<<"got Command : "<<command<<endl<<flush;
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
//   clientGaze.close();
  port.close();
  pos1->stop();
vel1->stop();
    robotHead.close();
//   port.interrupt();
//   port.close();
}
