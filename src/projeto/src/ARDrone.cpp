// ARDrone Control in C++
// by Pongsak Suvanpong psksvp@robotvision2.com psksvp@gmail.com
// based on JavaDrone http://code.google.com/p/javadrone/
// and droneController http://dronecontroller.codeplex.com/

#include <commonc++/DatagramSocket.h++>
#include <commonc++/Mutex.h++>
#include <commonc++/Thread.h++>
#include <commonc++/Common.h++>
#include <commonc++/Lock.h++>
#include <commonc++/String.h++>
#include <commonc++/ScopedLock.h++>



#include <string>
#include <sstream>
#include <vector>

#ifndef __APPLE__
#include "malloc.h"
#endif
#include "stdlib.h"

#include "ARDrone.h"


//////////////////////////////////////////////////
/////////////////////////////////////////////////////////
int nomain(int argc, char** argv)
{
  ARDrone::Controller Controller;
  ARDrone::NavigationData navData;
  //try
  //{
    Controller.connectWithDroneAtAddress("192.168.1.1");
    //ccxx::Thread::sleep(200);
    ARDrone::NavigationDataReceiver navDataReceiver(&Controller, "192.168.1.1");
    navDataReceiver.start();
    //ccxx::Thread::sleep(200);

    ARDrone::VideoDataReceiver vdoDataReceiver(&Controller, "192.168.1.1");
    vdoDataReceiver.start();
    c//cxx::Thread::sleep(200);

    Controller.switchToFrontCamera();
    ARDrone::VideoDecoder::Image image;
    for(int i = 0; i < 1000; i++)
    {
      Controller.sendWatchDogReset();

      //ccxx::Thread::sleep(200);

      navDataReceiver.copyDataTo(navData);
      std::cout << "->" << navData.orientation.pitch << ','
      << navData.orientation.roll << ','
      << navData.orientation.yaw << std::endl;

      vdoDataReceiver.copyDataTo(image);

      //std::cout << "w->" << width << ",h->" << height << std::endl;

    }
    navDataReceiver.stop();
    vdoDataReceiver.stop();
  }
  //catch(ccxx::Exception& ex)
  //{
  //  std::cout << ex.what() << std::endl;
  //}
  return 0;
}

////////////////////////////////////////////////
#define NAVDATA_DEMO_TAG (0)
#define NAVDATA_TIME_TAG (1)
#define NAVDATA_RAW_MEASURES_TAG (2)
#define NAVDATA_PHYS_MEASURES_TAG (3)
#define NAVDATA_GYROS_OFFSETS_TAG (4)
#define NAVDATA_EULER_ANGLES_TAG (5)
#define NAVDATA_REFERENCES_TAG (6)
#define NAVDATA_TRIMS_TAG (7)
#define NAVDATA_RC_REFERENCES_TAG (8)
#define NAVDATA_PWM_TAG (9)
#define NAVDATA_ALTITUDE_TAG (10)
#define NAVDATA_VISION_RAW_TAG (11)
#define NAVDATA_VISION_OF_TAG (12)
#define NAVDATA_VISION_TAG (13)
#define NAVDATA_VISION_PERF_TAG (14)
#define NAVDATA_TRACKERS_SEND_TAG (15)
#define NAVDATA_VISION_DETECT_TAG (16)
#define NAVDATA_WATCHDOG_TAG (17)
#define NAVDATA_ADC_DATA_FRAME_TAG (18)
#define NAVDATA_VIDEO_STREAM_TAG (19)
#define NAVDATA_CKS_TAG (0xFFFF)

/////////////////////////////////////////////

typedef float               float32_t;
typedef double              float64_t;



namespace ARDrone
{
  /////////////////////////////////////////////////////
  Drone::Drone()
  {
    myController = NULL;
    myVideoDataReceiver = NULL;
    myNavigationDataReceiver = NULL;
  }

  Drone::~Drone()
  {
    stop();
  }

  bool Drone::start(const char* szDroneAddress)
  {
    //try
    //{
      myController = new ARDrone::Controller;
      //myVideoDataReceiver = new ARDrone::VideoDataReceiver(myController,  szDroneAddress);
      myNavigationDataReceiver = new ARDrone::NavigationDataReceiver(myController,  szDroneAddress);


      myController->connectWithDroneAtAddress(szDroneAddress);
      //ccxx::Thread::sleep(200);

      myNavigationDataReceiver->start();
      //ccxx::Thread::sleep(200);

      //myVideoDataReceiver->start();
      //ccxx::Thread::sleep(200);
      return true;
    }
    //catch(ccxx::Exception& ex)
    //{
    //  std::cout << ex.what() << std::endl;
    //  delete myVideoDataReceiver;
    //  delete myNavigationDataReceiver;
    //  delete myController;
    //  return false;
    //}
  }

  void Drone::stop()
  {
    if(NULL == myNavigationDataReceiver)
      return;
    //try
   // {
      myNavigationDataReceiver->stop();
     // myVideoDataReceiver->stop();
   // }
   // catch(ccxx::Exception& ex)
    //{
    //  std::cout << ex.what() << std::endl;
    //}

   // if(NULL != myVideoDataReceiver)
    //  delete myVideoDataReceiver;
    if(NULL != myNavigationDataReceiver)
      delete myNavigationDataReceiver;
    if(NULL != myController)
      delete myController;

    myController = NULL;
   // myVideoDataReceiver = NULL;
    myNavigationDataReceiver = NULL;
  }

  ARDrone::Controller& Drone::controller()
  {
    return *myController;
  }

 // ARDrone::VideoDataReceiver& Drone::videoDataReceiver()
 // {
 //   return *myVideoDataReceiver;
 // }

  ARDrone::NavigationDataReceiver& Drone::navigationDataReceiver()
  {
    return *myNavigationDataReceiver;
  }
  ////////////////////////////////////////////////////
  const int kNavigationDataPort = 5554;
  const int kOnBoardVideoPort = 5555;
  const int kATCommandPort = 5556;


  //NavData offset
  const int kNavigationDataOffsetOfStateData    =  4;
  const int kNavigationDataOffsetOfBatteryData  = 24;
  const int kNavigationDataOffsetOfAltitudeData = 40;

  const unsigned int INTERVAL = 100;

  ///
  unsigned int myNextATSequence;
  unsigned int myLastATSequence;

  ///
  ATCommand myLastATCommand;

  ///////////////////////////////////////////////////////
  inline int floatToIntegerByteByByte(float f)
  {
    int result;
    ::memcpy(&result, &f, sizeof(int));
    return result;
  }
  ////////////////////////////////////////////////////////
  CommunicationChannel::CommunicationChannel()
  {
    myNextATSequence = 1;
    myLastATSequence = 1;
    myDatagram.init();
  }

  CommunicationChannel::~CommunicationChannel()
  {
    disconnectFromDrone();
  }

  void CommunicationChannel::connectWithDroneAtAddress(const char* szDroneIpAddress, int iPort)
  {
    ccxx::String strAddr(szDroneIpAddress);
    myDatagram.connect(strAddr, iPort);
    myDatagram.setTimeout(3000);
  }

  void CommunicationChannel::disconnectFromDrone()
  {
    if(isConnectedWithDrone())
    {
      myDatagram.shutdown();
    }
  }

  void CommunicationChannel::setTimeout(int t)
  {
    myDatagram.setTimeout(t);
  }

  bool CommunicationChannel::isConnectedWithDrone()
  {
    return myDatagram.isConnected();
  }

  void CommunicationChannel::send(unsigned char* bytes, unsigned int length)
  {
    synchronized(myMutex)
    {
      myDatagram.send(bytes, length);
    }
  }

  void CommunicationChannel::receive(unsigned char* bytes, unsigned int& bufferLength)
  {
    int actualReceivedLength = myDatagram.receive(bytes, bufferLength);
    bufferLength = actualReceivedLength;
  }

  void CommunicationChannel::sendAT(const char* szHeader, const char* szDetail, unsigned int mssleep)
  {
    {
      std::stringstream strStm;
      strStm << szHeader << nextATSequence() << szDetail << '\r';
      std::string strATCmd = strStm.str();
      myLastATCommand.strCommandHeader = szHeader;
      myLastATCommand.strCommandData = szDetail;
      //std::cout << "Sending AT command -> " << strATCmd << std::endl;
      myDatagram.send((unsigned char*)strATCmd.c_str(), strATCmd.length());
      if(mssleep > 0)
        ccxx::Thread::sleep(mssleep);
    }
  }


  unsigned int CommunicationChannel::nextATSequence()
  {
    ccxx::ScopedLock lock(myMutex);
    return myNextATSequence++;
  }

  ARDrone::ATCommand CommunicationChannel::lastATCommand()
  {
    ccxx::ScopedLock lock(myMutex);
    return myLastATCommand;
  }

  //////////////////////////////////////////////////
  Controller::Controller()
  {
  }

  Controller::~Controller()
  {
  }

  void Controller::connectWithDroneAtAddress(const char* szDroneIpAddress)
  {
    if(true == myCommunicationChannel.isConnectedWithDrone())
      return;
    myCommunicationChannel.connectWithDroneAtAddress(szDroneIpAddress, kATCommandPort);
    myCommunicationChannel.sendAT("AT*PMODE=", ",2");
    myCommunicationChannel.sendAT("AT*MISC=", ",2,20,2000,3000");
    myCommunicationChannel.sendAT("AT*REF=", ",290717696");
    myCommunicationChannel.sendAT("AT*COMWDG=", "");
    myCommunicationChannel.sendAT("AT*CONFIG=", ",\"control:altitude_max\",\"1000\""); //altitude max 1m
    myCommunicationChannel.sendAT("AT*CONFIG=", ",\"control:control_level\",\"0\""); //0:BEGINNER, 1:ACE, 2:MAX
    myCommunicationChannel.sendAT("AT*CONFIG=", ",\"general:navdata_demo\",\"TRUE\"");
    myCommunicationChannel.sendAT("AT*CONFIG=", ",\"general:video_enable\",\"TRUE\"");
    disableAdaptiveVideo();
    //myCommunicationChannel.sendAT("AT*CONFIG=", ",\"network:owner_mac\",\"00:18:DE:9D:E9:5D\""); //my PC
    //myCommunicationChannel.sendAT("AT*CONFIG=", ",\"network:owner_mac\",\"00:23:CD:5D:92:37\""); //AP
    myCommunicationChannel.sendAT("AT*CONFIG=", ",\"pic:ultrasound_freq\",\"8\"");
    myCommunicationChannel.sendAT("AT*FTRIM=", ""); //flat trim

    myCommunicationChannel.sendAT("AT*REF=", ",290717696");
    sendControlParameters(0, 0, 0, 0, 0);
    myCommunicationChannel.sendAT("AT*REF=", ",290717696");
    myCommunicationChannel.sendAT("AT*REF=", ",290717696");
  }

  void Controller::sendWatchDogReset()
  {
    myCommunicationChannel.sendAT("AT*COMWDG=", "");
  }

  void Controller::requestNavigationData()
  {
    myCommunicationChannel.sendAT("AT*CONFIG=", ",\"general:navdata_demo\",\"TRUE\"", 20);
  }

  void Controller::requestVideoData()
  {
    myCommunicationChannel.sendAT("AT*CONFIG=", ",\"general:video_enable\",\"TRUE\"", 10);
  }

  void Controller::sendFlatTrim()
  {
    myCommunicationChannel.sendAT("AT*FTRIM=", "");
  }

  void Controller::switchToFrontCamera()
  {
    myCommunicationChannel.sendAT("AT*ZAP=", "0");
  }

  void Controller::switchToDownCamera()
  {
    myCommunicationChannel.sendAT("AT*ZAP=", "2");
  }

  void Controller::takeOff()
  {
    myCommunicationChannel.sendAT("AT*REF=", ",290718208");
  }

  void Controller::land()
  {
    myCommunicationChannel.sendAT("AT*REF=", ",290717696");
  }

  void Controller::sendEmergencyShutdown()
  {
    myCommunicationChannel.sendAT("AT*REF=", ",290717952"); //toggle Emergency
  }

  void Controller::hover()
  {
    sendControlParameters(0, 0, 0, 0, 0);
  }

  void Controller::sendControlParameters(int enable, float pitch, float roll, float yaw, float gaz)
  {
    std::stringstream strStm;
    strStm << "," << enable << ','
           << floatToIntegerByteByByte(roll) << ','
           << floatToIntegerByteByByte(pitch) << ','
           << floatToIntegerByteByByte(gaz) << ','
           << floatToIntegerByteByByte(yaw);

    myCommunicationChannel.sendAT("AT*PCMD=", strStm.str().c_str());
  }

  void Controller::sendLastCommand()
  {
    ARDrone::ATCommand cmd = myCommunicationChannel.lastATCommand();
    myCommunicationChannel.sendAT(cmd.strCommandHeader.c_str(), cmd.strCommandData.c_str());
  }

  void Controller::disableAdaptiveVideo()
  {
    //ardrone_at_set_toy_configuration("video:bitrate_ctrl_mode","0")
    //"AT*CONFIG=%d,\"%s\",\"%s\"\r",

    myCommunicationChannel.sendAT("AT*CONFIG=", "video:bitrate_ctrl_mode,0");
  }

  /* ARDroneME --- Java (J2ME) based AR.Drone Controller
   Author: MAPGPS at
   http://www.ardrone-flyers.com/forum/viewforum.php?f=8
   http://www.rcgroups.com/forums/showthread.php?t=1401935
   https://projects.ardrone.org/projects/ardrone-api/boards
   http://www.ourdev.cn/bbs/bbs_list.jsp?bbs_id=1025
   http://bbs.5imx.com/bbs/viewthread.php?tid=415063
   Initial: 2011.03.13


   ########## Keyboad Layout ############
   Takeoff/Landing: a toggle button (or MediaPlayer button)
   Emergency: "E" button (or Camera button), only effective after Landing button pressed first)
   Hovering: when the Arrow button loosed
   Speed(%) slider: change rudder rate in range of 0%~90%
   Arrow Keys and 2 Soft-Joysticks on the touch screen are linked.

   Arrow Keys:
   Go Up
   ^
   |
   Go Left <---+---> Go Right
   |
   v
   Go Down

   Arrow Keys with central button pressed down (Shift):
   Go Forward
   ^
   |
   Rotate Left <--- ---> Rotate Right
   |
   v
   Go Backward

   UI_BIT:
   00010001010101000000000000000000
   |   | | | |        || | ||||+--0: Button turn to left
   |   | | | |        || | |||+---1: Button altitude down (ah - ab)
   |   | | | |        || | ||+----2: Button turn to right
   |   | | | |        || | |+-----3: Button altitude up (ah - ab)
   |   | | | |        || | +------4: Button - z-axis (r1 - l1)
   |   | | | |        || +--------6: Button + z-axis (r1 - l1)
   |   | | | |        |+----------8: Button emergency reset all
   |   | | | |        +-----------9: Button Takeoff / Landing
   |   | | | +-------------------18: y-axis trim +1 (Trim increase at +/- 1??/s)
   |   | | +---------------------20: x-axis trim +1 (Trim increase at +/- 1??/s)
   |   | +-----------------------22: z-axis trim +1 (Trim increase at +/- 1??/s)
   |   +-------------------------24: x-axis +1
   +-----------------------------28: y-axis +1

   AT*REF=<sequence>,<UI>
   AT*PCMD=<sequence>,<enable>,<roll>,<pitch>,<gaz>,<yaw>
   (float)0.05 = (int)1028443341           (float)-0.05 = (int)-1119040307
   (float)0.1  = (int)1036831949           (float)-0.1  = (int)-1110651699
   (float)0.2  = (int)1045220557           (float)-0.2  = (int)-1102263091
   (float)0.5  = (int)1056964608           (float)-0.5  = (int)-1090519040
   AT*ANIM=<sequence>,<animation>,<duration>
   AT*CONFIG=<sequence>,\"<name>\",\"<value>\"

   ########## AT Commands ############
   altitude max2m: AT*CONFIG=1,\"control:altitude_max\",\"2000\"   //10000=unlimited
   Takeoff:        AT*REF=1,290718208
   Landing:        AT*REF=1,290717696
   Hovering:       AT*PCMD=1,1,0,0,0,0
   gaz 0.1:        AT*PCMD=1,1,0,0,1036831949,0
   gaz -0.1:       AT*PCMD=1,1,0,0,-1110651699,0
   roll 0.1:       AT*PCMD=1,1,1036831949,0,0,0
   roll -0.1:      AT*PCMD=1,1,-1110651699,0,0,0
   yaw 0.1:        AT*PCMD=1,1,0,0,0,1036831949
   yaw -0.1:       AT*PCMD=1,1,0,0,0,-1110651699
   pitch 0.1:      AT*PCMD=1,1,0,1036831949,0,0
   pitch -0.1:     AT*PCMD=1,1,0,-1110651699,0,0
   pitch -30 deg:  AT*ANIM=1,0,1000
   pitch 30 deg:   AT*ANIM=1,1,1000
   Emergency       AT*REF=1,290717952
   Flat Trim:      AT*FTRIM=1
   */

  ///////////////////////////////////////////////
  NavigationDataReceiver::NavigationDataReceiver(ARDrone::Controller* pController, const char* szDroneIpAddress)
  {
    myDroneAddress = szDroneIpAddress;
    myController = pController;
  }

  NavigationDataReceiver::~NavigationDataReceiver() throw ()
  {
    if(true == isRunning())
    {
      try
      {
        stop();
        join();
        myCommunicationChannel.disconnectFromDrone();
      }
      catch (ccxx::Exception& ex)
      {
        std::cout << ex.what() << std::endl;
      }
    }
  }


  void NavigationDataReceiver::run()
  {
    std::cout << "NavigationDataReceiver started\n";
    try
    {
      myCommunicationChannel.connectWithDroneAtAddress(myDroneAddress.c_str(), kNavigationDataPort);
      myCommunicationChannel.setTimeout(3000);

      unsigned char trigger[4] = {0x01, 0x00, 0x00, 0x00};
      myCommunicationChannel.send(trigger, 4);



      unsigned char navDataDemo[10240];
      unsigned int navDataLength = 10240;
      while(false == testCancel())
      {
        try
        {
          synchronized(myMutex)
          {
            myController->requestNavigationData();
            myCommunicationChannel.receive(navDataDemo, navDataLength);

            MemoryLibrary::Buffer navDataBuffer(navDataDemo, navDataLength);
            parse(navDataBuffer);
          }
        }
        catch (ccxx::TimeoutException& timeoutEx)
        {
          std::cout << "NavigationDataReceiver TIMEOUT exception thrown.." << timeoutEx.what() << std::endl;
        }
        catch (ccxx::Exception& ex)
        {
          std::cout << "NavigationDataReceiver exception thrown.." << ex.what() << std::endl;
        }
      }//while
    }
    catch (ccxx::Exception& ex)
    {
      std::cout << "NavigationDataReceiver exception thrown.." << ex.what() << std::endl;
    }

    std::cout << "NavigationDataReceiver stopped\n";
  }
  ////////
  bool NavigationDataReceiver::parse(MemoryLibrary::Buffer& buffer)
  {
    int offset = 0;
    int header = buffer.MakeValueFromOffset<int32_t>(offset);
    if(header != 0x55667788)
    {
      std::cout << "NavigationDataReceiver FAIL, because the header != 0x55667788\n";
      return false;
    }

    offset += 4;
    int state = buffer.MakeValueFromOffset<int32_t>(offset);
    parseState(state);

    offset += 4;
    myNavData.sequence = buffer.MakeValueFromOffset<int32_t>(offset);

    offset += 4;
    // int vision_tag;

    offset += 4;
    while(offset < buffer.Size())
    {
      int option_tag = (int)buffer.MakeValueFromOffset<unsigned short>(offset);
      offset += 2;
      int option_len = (int)buffer.MakeValueFromOffset<unsigned short>(offset);
      offset += 2;

      if(option_len == 0)
      {
        std::cout << "NavigationDataReceiver FAIL, option_len == 0\n";
        return false;
      }

      switch(option_tag)
      {
        case NAVDATA_DEMO_TAG: parseNavigation(buffer, offset); break;
        case NAVDATA_CKS_TAG:  break;
        case NAVDATA_VISION_DETECT_TAG: parseVision(buffer, offset); break;
      }

      offset = offset + option_len - 4;
    } //while

    return true;
  }

  bool NavigationDataReceiver::parseState(int state)
  {
    myNavData.flying = (state & 1) != 0;
    myNavData.videoEnabled = (state & (1 << 1)) != 0;
    myNavData.visionEnabled = (state & (1 << 2)) != 0;
    myNavData.controlAlgorithm = (state & (1 << 3)) != 0 ? NavigationData::eAugularSpeedControl : NavigationData::eEulerAnglesControl;
    myNavData.altitudeControlActive = (state & (1 << 4)) != 0;
    myNavData.userFeedbackOn = (state & (1 << 5)) != 0;
    myNavData.controlReceived = (state & (1 << 6)) != 0;
    myNavData.trimReceived = (state & (1 << 7)) != 0;
    myNavData.trimRunning = (state & (1 << 8)) != 0;
    myNavData.trimSucceeded = (state & (1 << 9)) != 0;
    myNavData.navDataDemoOnly = (state & (1 << 10)) != 0;
    myNavData.navDataBootstrap = (state & (1 << 11)) != 0;
    myNavData.motorsDown = (state & (1 << 12)) != 0;
    myNavData.gyrometersDown = (state & (1 << 14)) != 0;
    myNavData.batteryTooLow = (state & (1 << 15)) != 0;
    myNavData.batteryTooHigh = (state & (1 << 16)) != 0;
    myNavData.timerElapsed = (state & (1 << 17)) != 0;
    myNavData.notEnoughPower = (state & (1 << 18)) != 0;
    myNavData.angelsOutOufRange = (state & (1 << 19)) != 0;
    myNavData.tooMuchWind = (state & (1 << 20)) != 0;
    myNavData.ultrasonicSensorDeaf = (state & (1 << 21)) != 0;
    myNavData.cutoutSystemDetected = (state & (1 << 22)) != 0;
    myNavData.PICVersionNumberOK = (state & (1 << 23)) != 0;
    myNavData.ATCodedThreadOn = (state & (1 << 24)) != 0;
    myNavData.navDataThreadOn = (state & (1 << 25)) != 0;
    myNavData.videoThreadOn = (state & (1 << 26)) != 0;
    myNavData.acquisitionThreadOn = (state & (1 << 27)) != 0;
    myNavData.controlWatchdogDelayed = (state & (1 << 28)) != 0;
    myNavData.ADCWatchdogDelayed = (state & (1 << 29)) != 0;
    myNavData.communicationProblemOccurred = (state & (1 << 30)) != 0;
    myNavData.emergency = (state & (1 << 31)) != 0;
    return true;
  }

  bool NavigationDataReceiver::parseNavigation(MemoryLibrary::Buffer& buffer, int offset)
  {
    int temp = buffer.MakeValueFromOffset<int32_t>(offset);
    myNavData.controlState = (NavigationData::eControlState)(temp >> 16);
    offset += 4;
    myNavData.batteryLevel = buffer.MakeValueFromOffset<int32_t>(offset);
    offset += 4;
    myNavData.orientation.pitch = buffer.MakeValueFromOffset<float32_t>(offset) / 1000.0f;
    offset += 4;
    myNavData.orientation.roll = buffer.MakeValueFromOffset<float32_t>(offset) / 1000.0f;
    offset += 4;
    myNavData.orientation.yaw = buffer.MakeValueFromOffset<float32_t>(offset) / 1000.0f;
    offset += 4;
    myNavData.altitude = (float)buffer.MakeValueFromOffset<int32_t>(offset) / 1000.0f;
    offset += 4;
    myNavData.speed.vx = buffer.MakeValueFromOffset<float32_t>(offset);
    offset += 4;
    myNavData.speed.vy = buffer.MakeValueFromOffset<float32_t>(offset);
    offset += 4;
    myNavData.speed.vz = buffer.MakeValueFromOffset<float32_t>(offset);
    offset += 4;

    return true;
  }

  bool NavigationDataReceiver::parseVision(MemoryLibrary::Buffer& buffer, int offset)
  {
    int nbDetected = buffer.MakeValueFromOffset<int32_t>(offset);
    offset += 4;

    if(0 == nbDetected) // not detecting anything
      return true;
    myNavData.visionTagVector.resize(0);
    for(int i = 0; i < nbDetected; i++)
    {
      ARDrone::NavigationData::VisionTag visionTag;

      visionTag.type = buffer.MakeValueFromOffset<int32_t>(offset + 4 * i);
      visionTag.x = buffer.MakeValueFromOffset<int32_t>(offset + 4 * i + 1 * nbDetected * 4);
      visionTag.y = buffer.MakeValueFromOffset<int32_t>(offset + 4 * i + 2 * nbDetected * 4);
      visionTag.width = buffer.MakeValueFromOffset<int32_t>(offset + 4 * i + 3 * nbDetected * 4);
      visionTag.height = buffer.MakeValueFromOffset<int32_t>(offset + 4 * i + 4 * nbDetected * 4);
      visionTag.distance = buffer.MakeValueFromOffset<int32_t>(offset + 4 * i + 5 * nbDetected * 4);

      myNavData.visionTagVector.push_back(visionTag);
    }
    return true;
  }

  void NavigationDataReceiver::copyDataTo(ARDrone::NavigationData& data)
  {
    synchronized(myMutex)
    {
      //::memcpy(&data, &myNavData, sizeof(ARDrone::NavigationData));
      data = myNavData;
    }
  }



 // namespace VideoDecoder

} // namespace ARDrone


