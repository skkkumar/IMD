#include <stdio.h>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
int main() {
    Network yarp; // set up yarp
//     BufferedPort<Bottle> targetPort;
//     targetPort.open("/mover/target/in");
    //Network::connect("/objectDetector/target","/mover/target/in");
//     Network::connect("/tracker/target:o","/mover/target/in");
    Property options;
    options.put("device", "remote_controlboard");
    options.put("local", "/mover/motor/client");
    options.put("remote", "/icubSim/head");
    PolyDriver robotHead(options);

    if (!robotHead.isValid()) 
    {
        printf("Cannot connect to robot head\n");
        return 1;
    }
    
    IPositionControl *pos;
    IVelocityControl *vel;
    IEncoders *enc;
    robotHead.view(pos);
    robotHead.view(vel);
    robotHead.view(enc);
    if (pos==NULL || vel==NULL || enc==NULL) 
    {
        printf("Cannot get interface to robot head\n");
        robotHead.close();
        return 1;
    }
    int jnts = 0;
    pos->getAxes(&jnts);
    Vector setpoints;
    setpoints.resize(jnts);
    bool Flag = false;
//     bool flag;
    while (1) 
    { // repeat forever
//         Bottle *target = targetPort.read(); // read a target
//         if (target!=NULL) 
//         { // check we actually got something
//             printf("We got a vector containing");
//             for (int i=0; i<target->size(); i++) 
//             {
//                 printf(" %g", target->get(i).asDouble());
//             }
//             printf("\n");
//             double x = target->get(0).asDouble();
//             double y = target->get(1).asDouble();
//             double conf = target->get(2).asDouble();
//             x -= 320/2;
//             y -= 240/2;
//             double vx = x*0.1;
//             double vy = -y*0.1;
            /* prepare command */
            for (int i=0; i<jnts; i++) 
            {
                setpoints[i] = 0;
            }
            
	    pos->positionMove(setpoints.data());
	    pos->checkMotionDone(&Flag);
	    while (!Flag){
	    pos->checkMotionDone(&Flag);}
//             if (conf>0.5) 
//             {
// 	      setpoints[1]=-45;
//                 setpoints[3] = 0;
                setpoints[4] = -45;
//             } 
//             else 
//             {
//                 setpoints[3] = 0;
//                 setpoints[4] = 0;
//             }
//             vel->velocityMove(setpoints.data());
	    pos->positionMove(setpoints.data());
	    pos->checkMotionDone(&Flag);
	    while (!Flag){
	    pos->checkMotionDone(&Flag);}
	    
	    
	    setpoints[4] = 45;
	    pos->positionMove(setpoints.data());
	    
	    pos->checkMotionDone(&Flag);
	    while (!Flag){
	    pos->checkMotionDone(&Flag);
        }
    }
    return 0;
}

