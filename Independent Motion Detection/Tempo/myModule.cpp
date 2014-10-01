

#include "myModule.h"
myModule::myModule()
{}

bool myModule::configure(ResourceFinder &rf)
{
  Time::turboBoost();
  manager_thr=new moveThread(rf);
        if (!manager_thr->start())
        {
            delete manager_thr;
            return false;
        }
  port_rpc_human.open("/objectDetection/human:io");
  port_rpc.open("/objectDetection/rpc");
  attach(port_rpc);
  return true;
}

bool myModule::interruptModule()
{
  manager_thr->interrupt();
  port_rpc_human.interrupt();
  port_rpc.interrupt();
  return true;
}


bool myModule::close()
{
  manager_thr->stop();
  delete manager_thr;
  port_rpc_human.close();
  port_rpc.close();
  return true;
}

bool myModule::respond(const Bottle &command)
{
  if(manager_thr->respComd(command))
    return true;
  else
    return false;
}

double myModule::getPeriod()    { return 1.0;  }
bool   myModule::updateModule()
{
  Bottle human_cmd,reply;
  port_rpc_human.read(human_cmd,true);
  if(human_cmd.size()>0)
  {
  if(manager_thr->respComd(human_cmd))
    return true;
  else
    return false;
  }

  return true;
}
