/**
 * #NoobTeam
 */

///Include libraries to comunicate using the Broker.
///To connect to a desire IP, run ./naocam --pip <adress> from ./toolchain/sdk/bin
#include <signal.h>
#include <boost/shared_ptr.hpp>
#include <alcommon/albroker.h>
#include <alcommon/almodule.h>
#include <alcommon/albrokermanager.h>
#include <alcommon/altoolsmain.h>

///Include the header file
#include "naocam.h"

///If the event is remote...
#ifdef NAOCAM_IS_REMOTE
# define ALCALL
#else
# ifdef _WIN32
#  define ALCALL __declspec(dllexport)
# else
#  define ALCALL
#  endif
#endif

extern "C"
{

///Create and run the module.
  ALCALL int _createModule(boost::shared_ptr<AL::ALBroker> pBroker)
  {
    // init broker with the main broker instance
    // from the parent executable
    AL::ALBrokerManager::setInstance(pBroker->fBrokerManager.lock());
    AL::ALBrokerManager::getInstance()->addBroker(pBroker);
    AL::ALModule::createModule<NaoCam>( pBroker, "NaoCam" );

    return 0;
  }
  
///Close the module.
  ALCALL int _closeModule()
  {
    return 0;
  }
}

#ifdef NAOCAM_IS_REMOTE
  int main(int argc, char *argv[])
  {
    /// Pointer to createModule
    TMainType sig;
    sig = &_createModule;
    /// Call main function.
    ALTools::mainFunction("naocam", argc, argv, sig);
  }
#endif

