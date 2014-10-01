
#include "observeThread.h"

observeThread::observeThread ( ResourceFinder &_rf ) : RateThread ( 20 ), rf ( _rf ) { }

bool observeThread::threadInit()
{
    setRate ( int ( 20.0 ) );
 printf("****** problem is not here ******");

    Property obGaze ( "(device gazecontrollerclient)" );
    obGaze.put ( "remote","/iKinGazeCtrl" );
    obGaze.put ( "local","/observe/gaze" );
    printf("****** problem is here ******");
    if ( !clientobsGaze.open ( obGaze ) )
    {
        printf ( "Problem opening the gaze" );
        return false;
    }
    // open the view
    clientobsGaze.view ( ogaze );

    // latch the controller context in order to preserve
    // it after closing the module
    // the context contains the tracking mode, the neck limits and so on.
    ogaze->storeContext ( &startup_context_id );

    // set trajectory time:
    ogaze->setNeckTrajTime ( 0.8 );
    ogaze->setEyesTrajTime ( 0.4 );
    i = 0;
    port.open ( "/observe/gaze/port" );
    return true;
}

void observeThread::run()
{
    if ( i == 1 )
    {
        ogaze->getJointsVelocities ( velVect );
        port.write ( velVect );
    }
}

void observeThread::interrupt()
{
    mutex.wait();
    port.interrupt();
    ogaze->stopControl();

    // it's a good rule to restore the controller
    // context as it was before opening the module
    ogaze->restoreContext ( startup_context_id );
    mutex.post();
}



bool observeThread::respComd ( const Bottle &Command )
{
    string command = Command.get ( 0 ).toString().c_str();
    if ( command == "MoveHead" )
    {
        i = 1;
        return true;
    }
    else if ( command == "StopHead" )
    {
        i = 0;
        return true;
    }

}

void observeThread::threadRelease()
{
    // we require an immediate stop
    // before closing the client for safety reason


    clientobsGaze.close();

//   port.interrupt();
    port.close();
}
