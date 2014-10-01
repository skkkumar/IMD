#include "moveThread.h"
moveThread::moveThread(ResourceFinder &_rf) : RateThread(20), rf(_rf) { }
bool moveThread::threadInit()
{
  setRate(int(20.0));
  port.open("/moveHead/check:o");
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
    jnts = 0;
    pos1->getAxes(&jnts);
    setpoints.resize(jnts);
    Flag = false;
  home();
  i = 0;
  return true;
}

void moveThread::Generate_motion(int port[]){
  for (int i = 0; i < sizeof(port)/4 ; i++){
	    setpoints[port[i]] = generate_rand_pos(eye_motion_speed_min,eye_motion_speed_max);
	    pos1->setRefSpeed(port[i], setpoints[port[i]]);
	    if (port[i] == 3){
	      setpoints[port[i]] = generate_rand_pos(eye_vertical_min, eye_vertical_max);
	    }
	    else if(port[i] == 4){
	      setpoints[port[i]] = generate_rand_pos(eye_horizontal_min, eye_horizontal_max);
	    }
      }
	    pos1->positionMove(setpoints.data());
	    pos1->checkMotionDone(&Flag);
	    while (!Flag){
	    pos1->checkMotionDone(&Flag);}
  
}

void moveThread::Generate_motion_constant(){
      setpoints[3] = eye_vertical_min+eye_vertical_adjustment;
      setpoints[4] = eye_horizontal_min+eye_horizontal_adjustment;
	    pos1->positionMove(setpoints.data());
	    pos1->checkMotionDone(&Flag);
	    while (!Flag){
	    pos1->checkMotionDone(&Flag);}
      setpoints[3] = eye_vertical_max-eye_vertical_adjustment;
      setpoints[4] = eye_horizontal_min+eye_horizontal_adjustment;
	    pos1->positionMove(setpoints.data());
	    pos1->checkMotionDone(&Flag);
	    while (!Flag){
	    pos1->checkMotionDone(&Flag);}
      setpoints[3] = eye_vertical_max-eye_vertical_adjustment;
      setpoints[4] = eye_horizontal_max-eye_horizontal_adjustment;
	    pos1->positionMove(setpoints.data());
	    pos1->checkMotionDone(&Flag);
	    while (!Flag){
	    pos1->checkMotionDone(&Flag);}
      setpoints[3] = eye_vertical_min+eye_vertical_adjustment;
      setpoints[4] = eye_horizontal_max-eye_horizontal_adjustment;
	    pos1->positionMove(setpoints.data());
	    pos1->checkMotionDone(&Flag);
	    while (!Flag){
	    pos1->checkMotionDone(&Flag);}
}


void moveThread::run()
{
  if (i == 1){
    int port[] = {3,4};
    Generate_motion(port);
  }
  else if (i == 2){
    Generate_motion_constant();
  }
  Message.addInt(1);
  port.write(Message);
}

int moveThread::generate_rand_pos(int min, int max)
{
srand(time(0));
return (rand() % (max - min) + min);
}



void moveThread::interrupt()
{
        mutex.wait();
        port.interrupt();
        mutex.post();
}

void moveThread::home(){
    for (i = 0; i < jnts; i++) {
  setpoints[i] = eye_motion_speed_min;
  pos1->setRefSpeed(i, setpoints[i]);
  }
            for (i=0; i<jnts; i++) 
            {
                setpoints[i] = 0;
            }
	    pos1->positionMove(setpoints.data());
	    pos1->checkMotionDone(&Flag);
	    while (!Flag){
	    pos1->checkMotionDone(&Flag);}
  
}
bool moveThread::respComd(const Bottle &Command){
  string command = Command.get(0).toString().c_str();
  if (command == "MoveHead"){
    i = 1;
    return true;
    }
    else if (command == "MoveHeadCon"){
      i = 2;
      return true;
    }
    else if (command == "StopHead"){
     home();
      i = 0;
      return true;
    }
}

void moveThread::threadRelease()
{
  port.close();
  pos1->stop();
  vel1->stop();
  robotHead.close();
}
