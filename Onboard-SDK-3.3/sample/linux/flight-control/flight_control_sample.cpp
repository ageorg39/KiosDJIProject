/*! @file flight_control_sample.cpp
 *  @version 3.3
 *  @date Jun 05 2017
 *
 *  @brief
 *  Flight Control API usage in a Linux environment.
 *  Provides a number of helpful additions to core API calls,
 *  especially for position control, attitude control, takeoff,
 *  landing.
 *
 *  @copyright
 *  2016-17 DJI. All rights reserved.
 * */

#include "flight_control_sample.hpp"
#include "telemetry_sample.hpp"
#include "mission_sample.hpp"
#include "ReadLiDAR.hpp"


using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;


void setupUSBfile(int USB){
	struct termios tty;
	struct termios tty_old;
	memset (&tty, 0, sizeof tty);

	/* Error Handling */
	if ( tcgetattr ( USB, &tty ) != 0 ) {
	   std::cout << "Error " << errno << " from tcgetattr: " << strerror(errno) << std::endl;
	}

	/* Save old tty parameters */
	tty_old = tty;

	/* Set Baud Rate */
	cfsetospeed (&tty, (speed_t)B9600);
	cfsetispeed (&tty, (speed_t)B9600);

	/* Setting other Port Stuff */
	tty.c_cflag     &=  ~PARENB;            // Make 8n1
	tty.c_cflag     &=  ~CSTOPB;
	tty.c_cflag     &=  ~CSIZE;
	tty.c_cflag     |=  CS8;

	tty.c_cflag     &=  ~CRTSCTS;           // no flow control
	tty.c_cc[VMIN]   =  1;                  // read doesn't block
	tty.c_cc[VTIME]  =  5;                  // 0.5 seconds read timeout
	tty.c_cflag     |=  CREAD | CLOCAL;     // turn on READ & ignore ctrl lines

	/* Make raw */
	cfmakeraw(&tty);

	/* Flush Port, then applies attributes */
	tcflush( USB, TCIFLUSH );
	if ( tcsetattr ( USB, TCSANOW, &tty ) != 0) {
	   std::cout << "Error " << errno << " from tcsetattr" << std::endl;
	}
}

float readAndSendArduinoDistance(int USB){
	int n = 0,
    spot = 0;
	char buf = '\0';
	/* Whole response*/
	char response[1024];
	float result = 0;
	memset(response, '\0', sizeof response);

	do {
		n = read( USB, &buf, 1 );
		sprintf( &response[spot], "%c", buf );
		spot += n;
	} while( buf != '\r' && n > 0);

	if (n < 0) {
		std::cout << "Error reading: " << strerror(errno) << std::endl;
		return -1;
	}
	else if (n == 0) {
		std::cout << "Read nothing!" << std::endl;
		return -1;
	}
	else {
	//	std::cout << "Response: " << response << std::endl;	
		result = atof(response);
		return result;
	}
}

//__


int
main(int argc, char** argv)
{
  // Initialize variables
  int functionTimeout = 1;

  // Setup OSDK.
  Vehicle* vehicle = setupOSDK(argc, argv);
  if (vehicle == NULL)
  {
    std::cout << "Vehicle not initialized, exiting.\n";
    return -1;
  }

  // Obtain Control Authority
  vehicle->obtainCtrlAuthority(functionTimeout);
   // Setup variables for use
  uint8_t wayptPolygonSides;
  int     hotptInitRadius;
  int     responseTimeout = 1; 
 bool userExitCommand = false;
  int ArduinoFileIN=0;  
  float distance = 0;
  
 while (!userExitCommand)
  {
  // Display interactive prompt
    std::cout << "\n\n========================================================" << std::endl;
    std::cout << "| Available commands:                                  |" << std::endl;
    std::cout << "| [a] Monitored Takeoff                                |" << std::endl;
    std::cout << "| [b] Landing                                          |" << std::endl;
    std::cout << "| [c] Get Data                                         |" << std::endl;
    std::cout << "| [d] Law Level Position Control                       |" << std::endl;
    std::cout << "| [e] High Level Mission                               |" << std::endl;
    std::cout << "| [p] Print Data from LiDAR                            |" << std::endl;
    std::cout << "| [n] Exit this sample                                 |" << std::endl;
    std::cout << "========================================================" << std::endl;
    std::cout << "Type your option:                                    " << std::endl;
 
  
  char inputChar;
  std::cin >> inputChar;

	switch (inputChar)
	{
		case 'a':
		{
		  monitoredTakeoff(vehicle); 
		  // Initialise home position
		  Telemetry::GlobalPosition HomePosition;
	      HomePosition = vehicle->broadcast->getGlobalPosition();
	      
	      std::cout << "\n Home Position (LLA)           = "
              << HomePosition.latitude << ", " << HomePosition.longitude
              << ", " << HomePosition.altitude << "\n\n"; 
		  }
		  break;
		  
		case 'b':
		{
		  monitoredLanding(vehicle);
	  }
		  break;
		case 'c':
		{		  
		        if (vehicle->getFwVersion() == Version::M100_31)
				{
					getBroadcastData(vehicle);
				}
			}
				break;
		case 'd':
		{
		Telemetry::GlobalPosition StartingPosition;
		Telemetry::GlobalPosition CurrentPosition;
		Telemetry::GlobalPosition TerminalPosition;
		float RemainDistance;
		
		StartingPosition = vehicle->broadcast->getGlobalPosition();
		CurrentPosition = StartingPosition;				
		TerminalPosition = StartingPosition;
		TerminalPosition.latitude = StartingPosition.latitude+0.000006;
		RemainDistance =TerminalPosition.latitude-CurrentPosition.latitude;
		
		moveByPositionOffset(vehicle, 0, 0, 10*2.435, 0);
		int i;	
		ArduinoFileIN = open("/dev/ttyACM1", O_RDWR | O_NOCTTY);
		setupUSBfile(ArduinoFileIN);
			i=1;
			while(i<=5){
				distance = readAndSendArduinoDistance(ArduinoFileIN)/100;
				i++;
			}
		printf ("Lidar measured distance %.2f\n" ,distance);
		
		while ( TerminalPosition.latitude-CurrentPosition.latitude >= 0.000001)
		{

			std::cout << "Remaining distance from target:"<< RemainDistance*1000000 << "\n"<< std::endl;			
			
			if ( distance <= 1){
		   	    moveByPositionOffset(vehicle, 6, 0, 2.44, 0);
			}
			else if ((distance <= 100)&&(distance >= 4)){
		   	    moveByPositionOffset(vehicle, 6, 0, -2.44, 0);
			}
			else 
			{ 
		      moveByPositionOffset(vehicle, 6, 0, 0, 0);
		    }
		   
		   CurrentPosition = vehicle->broadcast->getGlobalPosition();
			
	
        	 std::cout << "\n Current Position (LLA) = "
             	 << CurrentPosition.latitude << ", " << CurrentPosition.longitude
             	 << ", " << CurrentPosition.altitude << "\n\n";
          RemainDistance =TerminalPosition.latitude-CurrentPosition.latitude;
			setupUSBfile(ArduinoFileIN);
			i=1;
			while(i<=5)
			{
				distance = readAndSendArduinoDistance(ArduinoFileIN)/100;
				i++;
			}
			printf ("Lidar measured distance %.2f\n" ,distance);
		}
	}
		  
		break;

/*
	        std::cout << " [u] Up  \n [d] Down  \n [f] Forward  \n [b] Backward  \n [l] Left  \n [r] Right" << std::endl;	
			std::cout << "Type your option:                             " << std::endl;
			
			char direction;
			std::cin >> direction;
		  
		  switch (direction)
		  {  //		  moveByPositionOffset(vehicle, x, y, z, h);
			  case 'u':
				moveByPositionOffset(vehicle, 0, 0, 2.435, 0);				
				break;
			  
			  case 'd':
				moveByPositionOffset(vehicle, 0, 0, -2.435, 0);			  
				break;			  
			  
			  case 'f':
				moveByPositionOffset(vehicle, 5, 0, 0, 0);
				break;			  
			  
			  case 'b':
				moveByPositionOffset(vehicle, -5, 0, 0, 0);
				break;			  
			  case 'l':
				moveByPositionOffset(vehicle, 0, 5, 0, 0);			  
				break;			  
		 			  
			  case 'r':
				moveByPositionOffset(vehicle, 0, -5, 0, 0);
				break;			  
		  }
		  

		  break;
*/
		case 'e':
		{		  
		// Waypoint call
		wayptPolygonSides = 4;
		runWaypointMission(vehicle, wayptPolygonSides, responseTimeout);	
	}
		break;
	
		case 'p':
 {
        ArduinoFileIN = open("/dev/ttyACM0", O_RDWR | O_NOCTTY);
	
		setupUSBfile(ArduinoFileIN);
			int i;
			i=1;
			while(i<=10){
				distance = readAndSendArduinoDistance(ArduinoFileIN)/100;
				printf ("Lidar measured distance %.2f\n" ,distance);
				i++;
			}
	}
        break;
		
		
		case 'n':
		{
        userExitCommand = true;
	}
        break;
      
        default:
        {
			std::cout << "Invalid option entered - please enter a letter from the choices in the prompt.\n";
		}
			break;
	}
	usleep(1000000);
  }
  delete (vehicle);
  return 0;
}

/*! Monitored Takeoff (Blocking API call). Return status as well as ack.
    This version of takeoff makes sure your aircraft actually took off
    and only returns when takeoff is complete.
    Use unless you want to do other stuff during takeoff - this will block
    the main thread.
!*/
//==========================================================================

bool monitoredTakeoff(Vehicle* vehicle, int timeout)
{
  //@todo: remove this once the getErrorCode function signature changes
  char func[50];
  int  pkgIndex;

  if (vehicle->getFwVersion() != Version::M100_31)
  {
    // Telemetry: Verify the subscription
    ACK::ErrorCode subscribeStatus;
    subscribeStatus = vehicle->subscribe->verify(timeout);
    if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
    {
      ACK::getErrorCodeMessage(subscribeStatus, func);
      return false;
    }

    // Telemetry: Subscribe to flight status and mode at freq 10 Hz
    pkgIndex		      = 0;
    int       freq            = 10;
    TopicName topicList10Hz[] = { TOPIC_STATUS_FLIGHT,
                                  TOPIC_STATUS_DISPLAYMODE };
    int  numTopic        = sizeof(topicList10Hz) / sizeof(topicList10Hz[0]);
    bool enableTimestamp = false;

    bool pkgStatus = vehicle->subscribe->initPackageFromTopicList(
      pkgIndex, numTopic, topicList10Hz, enableTimestamp, freq);
    if (!(pkgStatus))
    {
      return pkgStatus;
    }
    subscribeStatus = vehicle->subscribe->startPackage(pkgIndex, timeout);
    if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
    {
      ACK::getErrorCodeMessage(subscribeStatus, func);
      // Cleanup before return
      vehicle->subscribe->removePackage(pkgIndex, timeout);
      return false;
    }
  }

  // Start takeoff
  ACK::ErrorCode takeoffStatus = vehicle->control->takeoff(timeout);
  if (ACK::getError(takeoffStatus) != ACK::SUCCESS)
  {
    ACK::getErrorCodeMessage(takeoffStatus, func);
    return false;
  }

  // First check: Motors started
  int motorsNotStarted = 0;
  int timeoutCycles    = 20;

  if (vehicle->getFwVersion() != Version::M100_31)
  {
    while (vehicle->subscribe->getValue<TOPIC_STATUS_FLIGHT>() !=
             VehicleStatus::FlightStatus::ON_GROUND &&
           vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() !=
             VehicleStatus::DisplayMode::MODE_ENGINE_START &&
           motorsNotStarted < timeoutCycles)
    {
      motorsNotStarted++;
      usleep(100000);
    }

    if (motorsNotStarted == timeoutCycles)
    {
      std::cout << "Takeoff failed. Motors are not spinning." << std::endl;
      // Cleanup
      if (vehicle->getFwVersion() != Version::M100_31)
      {
	vehicle->subscribe->removePackage(0, timeout);
      }
      return false;
    }
    else
    {
      std::cout << "Motors spinning...\n";
    }
  }
  else
  {
    while ((vehicle->broadcast->getStatus().flight !=
            DJI::OSDK::VehicleStatus::M100FlightStatus::TAKEOFF) &&
           motorsNotStarted < timeoutCycles)
    {
      motorsNotStarted++;
      usleep(100000);
    }

    if(motorsNotStarted == timeoutCycles)
    {
      std::cout << "Successful TakeOff!" << std::endl;
    }
  }

  // Second check: In air
  int stillOnGround = 0;
  timeoutCycles     = 110;

  if (vehicle->getFwVersion() != Version::M100_31)
  {
    while (vehicle->subscribe->getValue<TOPIC_STATUS_FLIGHT>() !=
             VehicleStatus::FlightStatus::IN_AIR &&
           (vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() !=
              VehicleStatus::DisplayMode::MODE_ASSISTED_TAKEOFF ||
            vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() !=
              VehicleStatus::DisplayMode::MODE_AUTO_TAKEOFF) &&
           stillOnGround < timeoutCycles)
    {
      stillOnGround++;
      usleep(100000);
    }

    if (stillOnGround == timeoutCycles)
    {
     std::cout << "Takeoff failed. Aircraft is still on the ground, but the "
		  "motors are spinning."
	       << std::endl;
     // Cleanup
     if (vehicle->getFwVersion() != Version::M100_31)
     {
       vehicle->subscribe->removePackage(0, timeout);
     }
     return false;
    }
    else
    {
     std::cout << "Ascending...\n";
    }
  }
  else
  {
    while ((vehicle->broadcast->getStatus().flight !=
            DJI::OSDK::VehicleStatus::M100FlightStatus::IN_AIR_STANDBY) &&
           stillOnGround < timeoutCycles)
    {
      stillOnGround++;
      usleep(100000);
    }

    if(stillOnGround == timeoutCycles)
    {
      std::cout << "Aircraft in air!" << std::endl;
    }
  }

  // Final check: Finished takeoff
  if (vehicle->getFwVersion() != Version::M100_31)
  {
    while (vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() ==
             VehicleStatus::DisplayMode::MODE_ASSISTED_TAKEOFF ||
           vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() ==
             VehicleStatus::DisplayMode::MODE_AUTO_TAKEOFF)
    {
      sleep(1);
    }

    if (vehicle->getFwVersion() != Version::M100_31)
    {
      if (vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() !=
	    VehicleStatus::DisplayMode::MODE_P_GPS ||
	  vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() !=
	    VehicleStatus::DisplayMode::MODE_ATTITUDE)
      {
	std::cout << "Successful takeoff!\n";
      }
      else
      {
	std::cout
	  << "Takeoff finished, but the aircraft is in an unexpected mode. "
	     "Please connect DJI GO.\n";
	vehicle->subscribe->removePackage(0, timeout);
	return false;
      }
    }
  }
  else
  {
    float32_t delta;
    Telemetry::GlobalPosition currentHeight; 
    Telemetry::GlobalPosition deltaHeight = vehicle->broadcast->getGlobalPosition();

    do
    {
      sleep(3);
      currentHeight = vehicle->broadcast->getGlobalPosition();
      delta         = fabs(currentHeight.altitude - deltaHeight.altitude);
      deltaHeight.altitude   = currentHeight.altitude;
    } while (delta >= 0.009);

    std::cout << "Aircraft hovering at " << currentHeight.altitude << "m!\n";
  }

  // Cleanup
  if (vehicle->getFwVersion() != Version::M100_31)
  {
    ACK::ErrorCode ack = vehicle->subscribe->removePackage(pkgIndex, timeout);
    if (ACK::getError(ack))
    {
      std::cout
        << "Error unsubscribing; please restart the drone/FC to get back "
           "to a clean state.\n";
    }
  }

  return true;
}
//===============================================================================
// Position Control
//===============================================================================
/*! Position Control. Allows you to set an offset from your current
    location. The aircraft will move to that position and stay there.
    Typical use would be as a building block in an outer loop that does not
    require many fast changes, perhaps a few-waypoint trajectory. For smoother
    transition and response you should convert your trajectory to attitude
    setpoints and use attitude control or convert to velocity setpoints
    and use velocity control.
!*/
int moveByPositionOffset(Vehicle* vehicle, float xOffsetDesired,
                     float yOffsetDesired, float zOffsetDesired,
                     float yawDesired, float posThresholdInM,
                     float yawThresholdInDeg)
{
  // Set timeout: this timeout is the time you allow the drone to take to finish
  // the
  // mission
  int responseTimeout              = 1;
  int timeoutInMilSec              = 10000;  // initially 10000
  int controlFreqInHz              = 50; // Hz
  int cycleTimeInMs                = 1000 / controlFreqInHz;
  int outOfControlBoundsTimeLimit  = 10 * cycleTimeInMs; // 10 cycles
  int withinControlBoundsTimeReqmt = 50 * cycleTimeInMs; // 50 cycles
  int pkgIndex;

  //@todo: remove this once the getErrorCode function signature changes
  char func[50];

  if (vehicle->getFwVersion() != Version::M100_31)
  {
    // Telemetry: Verify the subscription
    ACK::ErrorCode subscribeStatus;
    subscribeStatus = vehicle->subscribe->verify(responseTimeout);
    if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
    {
      ACK::getErrorCodeMessage(subscribeStatus, func);
      return false;
    }

    // Telemetry: Subscribe to quaternion, fused lat/lon and altitude at freq 50
    // Hz
    pkgIndex                  = 0;
    int       freq            = 50;
    TopicName topicList50Hz[] = { TOPIC_QUATERNION, TOPIC_GPS_FUSED };
    int       numTopic = sizeof(topicList50Hz) / sizeof(topicList50Hz[0]);
    bool      enableTimestamp = false;

    bool pkgStatus = vehicle->subscribe->initPackageFromTopicList(
      pkgIndex, numTopic, topicList50Hz, enableTimestamp, freq);
    if (!(pkgStatus))
    {
      return pkgStatus;
    }
    subscribeStatus =
      vehicle->subscribe->startPackage(pkgIndex, responseTimeout);
    if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
    {
      ACK::getErrorCodeMessage(subscribeStatus, func);
      // Cleanup before return
      vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
      return false;
    }
  }

  // Wait for data to come in
  sleep(1);

  // Get data

  // Global position retrieved via subscription
  Telemetry::TypeMap<TOPIC_GPS_FUSED>::type currentSubscriptionGPS;
  Telemetry::TypeMap<TOPIC_GPS_FUSED>::type originSubscriptionGPS;
  // Global position retrieved via broadcast
  Telemetry::GlobalPosition currentBroadcastGP;
  Telemetry::GlobalPosition originBroadcastGP;

  // Convert position offset from first position to local coordinates
  Telemetry::Vector3f localOffset;

  if (vehicle->getFwVersion() != Version::M100_31)
  {
    currentSubscriptionGPS = vehicle->subscribe->getValue<TOPIC_GPS_FUSED>();
    originSubscriptionGPS  = currentSubscriptionGPS;
    localOffsetFromGpsOffset(vehicle, localOffset,
                             static_cast<void*>(&currentSubscriptionGPS),
                             static_cast<void*>(&originSubscriptionGPS));
  }
  else
  {
    currentBroadcastGP = vehicle->broadcast->getGlobalPosition();
    originBroadcastGP  = currentBroadcastGP;
    localOffsetFromGpsOffset(vehicle, localOffset,
                             static_cast<void*>(&currentBroadcastGP),
                             static_cast<void*>(&originBroadcastGP));
  }

  // Get initial offset. We will update this in a loop later.
  double xOffsetRemaining = xOffsetDesired - localOffset.x;
  double yOffsetRemaining = yOffsetDesired - localOffset.y;
  double zOffsetRemaining = zOffsetDesired - (-localOffset.z);

  // Conversions
  double yawDesiredRad     = DEG2RAD * yawDesired;
  double yawThresholdInRad = DEG2RAD * yawThresholdInDeg;

  //! Get Euler angle

  // Quaternion retrieved via subscription
  Telemetry::TypeMap<TOPIC_QUATERNION>::type subscriptionQ;
  // Quaternion retrieved via broadcast
  Telemetry::Quaternion broadcastQ;

  double yawInRad;
  if (vehicle->getFwVersion() != Version::M100_31)
  {
    subscriptionQ = vehicle->subscribe->getValue<TOPIC_QUATERNION>();
    yawInRad = toEulerAngle((static_cast<void*>(&subscriptionQ))).z / DEG2RAD;
  }
  else
  {
    broadcastQ = vehicle->broadcast->getQuaternion();
    yawInRad   = toEulerAngle((static_cast<void*>(&broadcastQ))).z / DEG2RAD;
  }

  int   elapsedTimeInMs     = 0;
  int   withinBoundsCounter = 0;
  int   outOfBounds         = 0;
  int   brakeCounter        = 0;
  int   speedFactor         = 1;
  float xCmd, yCmd, zCmd;
  // There is a deadband in position control
  // the z cmd is absolute height
  // while x and y are in relative
  float zDeadband = 0.12;

  if(vehicle->getFwVersion() == Version::M100_31)
  {
    zDeadband = 0.12 * 10;
  }

  /*! Calculate the inputs to send the position controller. We implement basic
   *  receding setpoint position control and the setpoint is always 1 m away
   *  from the current position - until we get within a threshold of the goal.
   *  From that point on, we send the remaining distance as the setpoint.
   */
  if (xOffsetDesired > 0)
    xCmd = (xOffsetDesired < speedFactor) ? xOffsetDesired : speedFactor;
  else if (xOffsetDesired < 0)
    xCmd =
      (xOffsetDesired > -1 * speedFactor) ? xOffsetDesired : -1 * speedFactor;
  else
    xCmd = 0;

  if (yOffsetDesired > 0)
    yCmd = (yOffsetDesired < speedFactor) ? yOffsetDesired : speedFactor;
  else if (yOffsetDesired < 0)
    yCmd =
      (yOffsetDesired > -1 * speedFactor) ? yOffsetDesired : -1 * speedFactor;
  else
    yCmd = 0;

  if (vehicle->getFwVersion() != Version::M100_31)
  {
    zCmd = currentSubscriptionGPS.altitude + zOffsetDesired;
  }
  else
  {
    zCmd = currentBroadcastGP.altitude + zOffsetDesired;
  }

  //! Main closed-loop receding setpoint position control
  while (elapsedTimeInMs < timeoutInMilSec)
  {

		vehicle->control->positionAndYawCtrl(xCmd, yCmd, zCmd,
											 yawDesiredRad / DEG2RAD);

		usleep(cycleTimeInMs * 1000);
		elapsedTimeInMs += cycleTimeInMs;

		//! Get current position in required coordinates and units
		if (vehicle->getFwVersion() != Version::M100_31)
		{
		  subscriptionQ = vehicle->subscribe->getValue<TOPIC_QUATERNION>();
		  yawInRad      = toEulerAngle((static_cast<void*>(&subscriptionQ))).z;
		  currentSubscriptionGPS = vehicle->subscribe->getValue<TOPIC_GPS_FUSED>();
		  localOffsetFromGpsOffset(vehicle, localOffset,
								   static_cast<void*>(&currentSubscriptionGPS),
								   static_cast<void*>(&originSubscriptionGPS));
		}
		else
		{
		  broadcastQ         = vehicle->broadcast->getQuaternion();
		  yawInRad           = toEulerAngle((static_cast<void*>(&broadcastQ))).z;
		  currentBroadcastGP = vehicle->broadcast->getGlobalPosition();
		  localOffsetFromGpsOffset(vehicle, localOffset,
								   static_cast<void*>(&currentBroadcastGP),
								   static_cast<void*>(&originBroadcastGP));
		}

		//! See how much farther we have to go
		xOffsetRemaining = xOffsetDesired - localOffset.x;
		yOffsetRemaining = yOffsetDesired - localOffset.y;
		zOffsetRemaining = zOffsetDesired - (-localOffset.z);

		//! See if we need to modify the setpoint
		if (std::abs(xOffsetRemaining) < speedFactor)
		  xCmd = xOffsetRemaining;
		if (std::abs(yOffsetRemaining) < speedFactor)
		  yCmd = yOffsetRemaining;

		if(vehicle->getFwVersion() == Version::M100_31 &&
		   std::abs(xOffsetRemaining) < posThresholdInM &&
		   std::abs(yOffsetRemaining) < posThresholdInM &&
		   std::abs(yawInRad - yawDesiredRad) < yawThresholdInRad)
		{
		  //! 1. We are within bounds; start incrementing our in-bound counter
		  withinBoundsCounter += cycleTimeInMs;
		}
		else if(std::abs(xOffsetRemaining) < posThresholdInM &&
		   std::abs(yOffsetRemaining) < posThresholdInM &&
		   std::abs(zOffsetRemaining) < zDeadband &&
		   std::abs(yawInRad - yawDesiredRad) < yawThresholdInRad)
		{
		  //! 1. We are within bounds; start incrementing our in-bound counter
		  withinBoundsCounter += cycleTimeInMs;
		}
		else
		{
		  if (withinBoundsCounter != 0)
		  {
			//! 2. Start incrementing an out-of-bounds counter
			outOfBounds += cycleTimeInMs;
		  }
		}
		//! 3. Reset withinBoundsCounter if necessary
		if (outOfBounds > outOfControlBoundsTimeLimit)
		{
		  withinBoundsCounter = 0;
		  outOfBounds         = 0;
		}
		//! 4. If within bounds, set flag and break
		if (withinBoundsCounter >= withinControlBoundsTimeReqmt)
		{
		  break;
		}
  }

  //! Set velocity to zero, to prevent any residual velocity from position
  //! command
  if (vehicle->getFwVersion() != Version::M100_31)
  {
    while (brakeCounter < withinControlBoundsTimeReqmt)
    {
      vehicle->control->emergencyBrake();
      usleep(cycleTimeInMs);
      brakeCounter += cycleTimeInMs;
    }
  }

  if (elapsedTimeInMs >= timeoutInMilSec)
  {
    std::cout << "Task timeout!\n";
    if (vehicle->getFwVersion() != Version::M100_31)
    {
      ACK::ErrorCode ack =
        vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
      if (ACK::getError(ack))
      {
        std::cout << "Error unsubscribing; please restart the drone/FC to get "
                     "back to a clean state.\n";
      }
    }
    return ACK::FAIL;
  }

  if (vehicle->getFwVersion() != Version::M100_31)
  {
    ACK::ErrorCode ack =
      vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
    if (ACK::getError(ack))
    {
      std::cout
        << "Error unsubscribing; please restart the drone/FC to get back "
           "to a clean state.\n";
    }
  }

  return ACK::SUCCESS;
}
//======================================================================
// Landing
//======================================================================
/*! Monitored Takeoff (Blocking API call). Return status as well as ack.
    This version of takeoff makes sure your aircraft actually took off
    and only returns when takeoff is complete.
    Use unless you want to do other stuff during takeoff - this will block
    the main thread.
!*/
bool
monitoredLanding(Vehicle* vehicle, int timeout)
{
  //@todo: remove this once the getErrorCode function signature changes
  char func[50];
  int  pkgIndex;

  if (vehicle->getFwVersion() != Version::M100_31)
  {
    // Telemetry: Verify the subscription
    ACK::ErrorCode subscribeStatus;
    subscribeStatus = vehicle->subscribe->verify(timeout);
    if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
    {
      ACK::getErrorCodeMessage(subscribeStatus, func);
      return false;
    }

    // Telemetry: Subscribe to flight status and mode at freq 10 Hz
    pkgIndex                  = 0;
    int       freq            = 10;
    TopicName topicList10Hz[] = { TOPIC_STATUS_FLIGHT,
                                  TOPIC_STATUS_DISPLAYMODE };
    int  numTopic        = sizeof(topicList10Hz) / sizeof(topicList10Hz[0]);
    bool enableTimestamp = false;

    bool pkgStatus = vehicle->subscribe->initPackageFromTopicList(
      pkgIndex, numTopic, topicList10Hz, enableTimestamp, freq);
    if (!(pkgStatus))
    {
      return pkgStatus;
    }
    subscribeStatus = vehicle->subscribe->startPackage(pkgIndex, timeout);
    if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
    {
      ACK::getErrorCodeMessage(subscribeStatus, func);
      // Cleanup before return
      vehicle->subscribe->removePackage(pkgIndex, timeout);
      return false;
    }
  }

  // Start landing
  ACK::ErrorCode landingStatus = vehicle->control->land(timeout);
  if (ACK::getError(landingStatus) != ACK::SUCCESS)
  {
    ACK::getErrorCodeMessage(landingStatus, func);
    return false;
  }

  // First check: Landing started
  int landingNotStarted = 0;
  int timeoutCycles     = 20;

  if (vehicle->getFwVersion() != Version::M100_31)
  {
    while (vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() !=
             VehicleStatus::DisplayMode::MODE_AUTO_LANDING &&
           landingNotStarted < timeoutCycles)
    {
      landingNotStarted++;
      usleep(100000);
    }
  }
  else
  {
    while (vehicle->broadcast->getStatus().flight !=
             DJI::OSDK::VehicleStatus::M100FlightStatus::LANDING &&
           landingNotStarted < timeoutCycles)
    {
      landingNotStarted++;
      usleep(100000);
    }
  }

  if (landingNotStarted == timeoutCycles)
  {
    std::cout << "Landing failed. Aircraft is still in the air." << std::endl;
    // Cleanup before return
    ACK::ErrorCode ack = vehicle->subscribe->removePackage(pkgIndex, timeout);
    if (ACK::getError(ack))
    {
      std::cout << "Error unsubscribing; please restart the drone/FC to get "
                   "back to a clean state.\n";
    }
    return false;
  }
  else
  {
    std::cout << "Landing...\n";
  }

  // Second check: Finished landing
  if (vehicle->getFwVersion() != Version::M100_31)
  {
    while (vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() ==
             VehicleStatus::DisplayMode::MODE_AUTO_LANDING &&
           vehicle->subscribe->getValue<TOPIC_STATUS_FLIGHT>() ==
             VehicleStatus::FlightStatus::IN_AIR)
    {
      sleep(1);
    }

    if (vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() !=
          VehicleStatus::DisplayMode::MODE_P_GPS ||
        vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() !=
          VehicleStatus::DisplayMode::MODE_ATTITUDE)
    {
      std::cout << "Successful landing!\n";
    }
    else
    {
      std::cout
        << "Landing finished, but the aircraft is in an unexpected mode. "
           "Please connect DJI GO.\n";
      ACK::ErrorCode ack = vehicle->subscribe->removePackage(pkgIndex, timeout);
      if (ACK::getError(ack))
      {
        std::cout << "Error unsubscribing; please restart the drone/FC to get "
                     "back to a clean state.\n";
      }
      return false;
    }
  }
  else
  {
    while (vehicle->broadcast->getStatus().flight ==
           DJI::OSDK::VehicleStatus::M100FlightStatus::FINISHING_LANDING)
    {
      sleep(1);
    }

    Telemetry::GlobalPosition gp;
    do
    {
      sleep(2);
      gp = vehicle->broadcast->getGlobalPosition();
    } while (gp.altitude != 0);

    if(gp.altitude != 0)
    {
      std::cout
	<< "Landing finished, but the aircraft is in an unexpected mode. "
	   "Please connect DJI GO.\n";
      return false;
    }
    else
    {
      std::cout << "Successful landing!\n";
    }
  }

  // Cleanup
  if (vehicle->getFwVersion() != Version::M100_31)
  {
    ACK::ErrorCode ack = vehicle->subscribe->removePackage(pkgIndex, timeout);
    if (ACK::getError(ack))
    {
      std::cout
        << "Error unsubscribing; please restart the drone/FC to get back "
           "to a clean state.\n";
    }
  }

  return true;
}
//======================================================================
// Helper Functions
//======================================================================
/*! Very simple calculation of local NED offset between two pairs of GPS
/coordinates.
    Accurate when distances are small.
!*/

void localOffsetFromGpsOffset(Vehicle* vehicle, Telemetry::Vector3f& deltaNed,
                         void* target, void* origin)
{
  Telemetry::GPSFused*       subscriptionTarget;
  Telemetry::GPSFused*       subscriptionOrigin;
  Telemetry::GlobalPosition* broadcastTarget;
  Telemetry::GlobalPosition* broadcastOrigin;
  double                     deltaLon;
  double                     deltaLat;

  if (vehicle->getFwVersion() != Version::M100_31)
  {
    subscriptionTarget = (Telemetry::GPSFused*)target;
    subscriptionOrigin = (Telemetry::GPSFused*)origin;
    deltaLon   = subscriptionTarget->longitude - subscriptionOrigin->longitude;
    deltaLat   = subscriptionTarget->latitude - subscriptionOrigin->latitude;
    deltaNed.x = deltaLat * C_EARTH;
    deltaNed.y = deltaLon * C_EARTH * cos(subscriptionTarget->latitude);
    deltaNed.z = subscriptionTarget->altitude - subscriptionOrigin->altitude;
  }
  else
  {
    broadcastTarget = (Telemetry::GlobalPosition*)target;
    broadcastOrigin = (Telemetry::GlobalPosition*)origin;
    deltaLon        = broadcastTarget->longitude - broadcastOrigin->longitude;
    deltaLat        = broadcastTarget->latitude - broadcastOrigin->latitude;
    deltaNed.x      = deltaLat * C_EARTH;
    deltaNed.y      = deltaLon * C_EARTH * cos(broadcastTarget->latitude);
    deltaNed.z      = broadcastTarget->altitude - broadcastOrigin->altitude;
  }
}

Telemetry::Vector3f
toEulerAngle(void* quaternionData)
{
  Telemetry::Vector3f    ans;
  Telemetry::Quaternion* quaternion = (Telemetry::Quaternion*)quaternionData;

  double q2sqr = quaternion->q2 * quaternion->q2;
  double t0    = -2.0 * (q2sqr + quaternion->q3 * quaternion->q3) + 1.0;
  double t1 =
    +2.0 * (quaternion->q1 * quaternion->q2 + quaternion->q0 * quaternion->q3);
  double t2 =
    -2.0 * (quaternion->q1 * quaternion->q3 - quaternion->q0 * quaternion->q2);
  double t3 =
    +2.0 * (quaternion->q2 * quaternion->q3 + quaternion->q0 * quaternion->q1);
  double t4 = -2.0 * (quaternion->q1 * quaternion->q1 + q2sqr) + 1.0;

  t2 = (t2 > 1.0) ? 1.0 : t2;
  t2 = (t2 < -1.0) ? -1.0 : t2;

  ans.x = asin(t2);
  ans.y = atan2(t3, t4);
  ans.z = atan2(t1, t0);

  return ans;
}

//======================================================================
// Print Data
//======================================================================
bool getBroadcastData(DJI::OSDK::Vehicle* vehicle, int responseTimeout)
{
  // Counters
  int elapsedTimeInMs = 0;
  int timeToPrintInMs = 1;

  // We will listen to five broadcast data sets:
  // 1. Flight Status
  // 2. Global Position
  // 3. RC Channels
  // 4. Velocity
  // 5. Quaternion

  // Please make sure your drone is in simulation mode. You can
  // fly the drone with your RC to get different values.

  Telemetry::Status         status;
  Telemetry::GlobalPosition globalPosition;
  Telemetry::RC             rc;
  Telemetry::Vector3f       velocity;
  Telemetry::Quaternion     quaternion;

  const int TIMEOUT = 20;

  // Re-set Broadcast frequencies to their default values
  ACK::ErrorCode ack = vehicle->broadcast->setBroadcastFreqDefaults(TIMEOUT);

  // Print in a loop for 2 seconds
  while (elapsedTimeInMs < timeToPrintInMs)
  {
    // Matrice 100 broadcasts only flight status
    status         = vehicle->broadcast->getStatus();
    globalPosition = vehicle->broadcast->getGlobalPosition();
    rc             = vehicle->broadcast->getRC();
    velocity       = vehicle->broadcast->getVelocity();
    quaternion     = vehicle->broadcast->getQuaternion();

    std::cout << "Counter = " << elapsedTimeInMs << ":\n";
    std::cout << "-------\n";
    std::cout << "Flight Status                         = "
              << (unsigned)status.flight << "\n";
    std::cout << "Position              (LLA)           = "
              << globalPosition.latitude << ", " << globalPosition.longitude
              << ", " << globalPosition.altitude << "\n";
    std::cout << "RC Commands           (r/p/y/thr)     = " << rc.roll << ", "
              << rc.pitch << ", " << rc.yaw << ", " << rc.throttle << "\n";
    std::cout << "Velocity              (vx,vy,vz)      = " << velocity.x
              << ", " << velocity.y << ", " << velocity.z << "\n";
    std::cout << "Attitude Quaternion   (w,x,y,z)       = " << quaternion.q0
              << ", " << quaternion.q1 << ", " << quaternion.q2 << ", "
              << quaternion.q3 << "\n";
    std::cout << "-------\n\n";

    usleep(5000);
    elapsedTimeInMs += 1;
  }

  std::cout << "Done printing!\n";
  return true;
}

bool subscribeToData(Vehicle* vehicle, int responseTimeout)
{

  // Counters
  int elapsedTimeInMs = 0;
  int timeToPrintInMs = 2000;

  // We will subscribe to six kinds of data:
  // 1. Flight Status at 1 Hz
  // 2. Fused Lat/Lon at 10Hz
  // 3. Fused Altitude at 10Hz
  // 4. RC Channels at 50 Hz
  // 5. Velocity at 50 Hz
  // 6. Quaternion at 200 Hz

  // Please make sure your drone is in simulation mode. You can fly the drone
  // with your RC to
  // get different values.

  // Telemetry: Verify the subscription
  ACK::ErrorCode subscribeStatus;
  subscribeStatus = vehicle->subscribe->verify(responseTimeout);
  if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
  {
    ACK::getErrorCodeMessage(subscribeStatus, __func__);
    return false;
  }

  // Package 0: Subscribe to flight status at freq 1 Hz
  int       pkgIndex        = 0;
  int       freq            = 1;
  TopicName topicList1Hz[]  = { TOPIC_STATUS_FLIGHT };
  int       numTopic        = sizeof(topicList1Hz) / sizeof(topicList1Hz[0]);
  bool      enableTimestamp = false;

  bool pkgStatus = vehicle->subscribe->initPackageFromTopicList(
    pkgIndex, numTopic, topicList1Hz, enableTimestamp, freq);
  if (!(pkgStatus))
  {
    return pkgStatus;
  }
  subscribeStatus = vehicle->subscribe->startPackage(pkgIndex, responseTimeout);
  if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
  {
    ACK::getErrorCodeMessage(subscribeStatus, __func__);
    // Cleanup before return
    vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
    return false;
  }

  // Package 1: Subscribe to Lat/Lon, and Alt at freq 10 Hz
  pkgIndex                  = 1;
  freq                      = 10;
  TopicName topicList10Hz[] = { TOPIC_GPS_FUSED, TOPIC_ALTITUDE_FUSIONED };
  numTopic                  = sizeof(topicList10Hz) / sizeof(topicList10Hz[0]);
  enableTimestamp           = false;

  pkgStatus = vehicle->subscribe->initPackageFromTopicList(
    pkgIndex, numTopic, topicList10Hz, enableTimestamp, freq);
  if (!(pkgStatus))
  {
    return pkgStatus;
  }
  subscribeStatus = vehicle->subscribe->startPackage(pkgIndex, responseTimeout);
  if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
  {
    ACK::getErrorCodeMessage(subscribeStatus, __func__);
    // Cleanup before return
    vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
    return false;
  }

  // Package 2: Subscribe to RC Channel and Velocity at freq 50 Hz
  pkgIndex                  = 2;
  freq                      = 50;
  TopicName topicList50Hz[] = { TOPIC_RC, TOPIC_VELOCITY };
  numTopic                  = sizeof(topicList50Hz) / sizeof(topicList50Hz[0]);
  enableTimestamp           = false;

  pkgStatus = vehicle->subscribe->initPackageFromTopicList(
    pkgIndex, numTopic, topicList50Hz, enableTimestamp, freq);
  if (!(pkgStatus))
  {
    return pkgStatus;
  }
  subscribeStatus = vehicle->subscribe->startPackage(pkgIndex, responseTimeout);
  if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
  {
    ACK::getErrorCodeMessage(subscribeStatus, __func__);
    // Cleanup before return
    vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
    return false;
  }

  // Package 3: Subscribe to Quaternion at freq 200 Hz.
  pkgIndex                   = 3;
  freq                       = 200;
  TopicName topicList200Hz[] = { TOPIC_QUATERNION };
  numTopic        = sizeof(topicList200Hz) / sizeof(topicList200Hz[0]);
  enableTimestamp = false;

  pkgStatus = vehicle->subscribe->initPackageFromTopicList(
    pkgIndex, numTopic, topicList200Hz, enableTimestamp, freq);
  if (!(pkgStatus))
  {
    return pkgStatus;
  }
  subscribeStatus = vehicle->subscribe->startPackage(pkgIndex, responseTimeout);
  if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
  {
    ACK::getErrorCodeMessage(subscribeStatus, __func__);
    // Cleanup before return
    vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
    return false;
  }

  // Wait for the data to start coming in.
  sleep(1);

  // Get all the data once before the loop to initialize vars
  TypeMap<TOPIC_STATUS_FLIGHT>::type     flightStatus;
  TypeMap<TOPIC_GPS_FUSED>::type         latLon;
  TypeMap<TOPIC_ALTITUDE_FUSIONED>::type altitude;
  TypeMap<TOPIC_RC>::type                rc;
  TypeMap<TOPIC_VELOCITY>::type          velocity;
  TypeMap<TOPIC_QUATERNION>::type        quaternion;

  // Print in a loop for 2 sec
  while (elapsedTimeInMs < timeToPrintInMs)
  {
    flightStatus = vehicle->subscribe->getValue<TOPIC_STATUS_FLIGHT>();
    latLon       = vehicle->subscribe->getValue<TOPIC_GPS_FUSED>();
    altitude     = vehicle->subscribe->getValue<TOPIC_ALTITUDE_FUSIONED>();
    rc           = vehicle->subscribe->getValue<TOPIC_RC>();
    velocity     = vehicle->subscribe->getValue<TOPIC_VELOCITY>();
    quaternion   = vehicle->subscribe->getValue<TOPIC_QUATERNION>();

    std::cout << "Counter = " << elapsedTimeInMs << ":\n";
    std::cout << "-------\n";
    std::cout << "Flight Status                         = " << (int)flightStatus
              << "\n";
    std::cout << "Position              (LLA)           = " << latLon.latitude
              << ", " << latLon.longitude << ", " << altitude << "\n";
    std::cout << "RC Commands           (r/p/y/thr)     = " << rc.roll << ", "
              << rc.pitch << ", " << rc.yaw << ", " << rc.throttle << "\n";
    std::cout << "Velocity              (vx,vy,vz)      = " << velocity.data.x
              << ", " << velocity.data.y << ", " << velocity.data.z << "\n";
    std::cout << "Attitude Quaternion   (w,x,y,z)       = " << quaternion.q0
              << ", " << quaternion.q1 << ", " << quaternion.q2 << ", "
              << quaternion.q3 << "\n";
    std::cout << "-------\n\n";

    usleep(5000);
    elapsedTimeInMs += 5;
  }

  std::cout << "Done printing!\n";
  vehicle->subscribe->removePackage(0, responseTimeout);
  vehicle->subscribe->removePackage(1, responseTimeout);
  vehicle->subscribe->removePackage(2, responseTimeout);
  vehicle->subscribe->removePackage(3, responseTimeout);

  return true;
}

//=================================================================================

bool setUpSubscription(DJI::OSDK::Vehicle* vehicle, int responseTimeout)
{
  // Telemetry: Verify the subscription
  ACK::ErrorCode subscribeStatus;

  subscribeStatus = vehicle->subscribe->verify(responseTimeout);
  if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
  {
    ACK::getErrorCodeMessage(subscribeStatus, __func__);
    return false;
  }

  // Telemetry: Subscribe to flight status and mode at freq 10 Hz
  int       freq            = 10;
  TopicName topicList10Hz[] = { TOPIC_GPS_FUSED };
  int       numTopic        = sizeof(topicList10Hz) / sizeof(topicList10Hz[0]);
  bool      enableTimestamp = false;

  bool pkgStatus = vehicle->subscribe->initPackageFromTopicList(
    DEFAULT_PACKAGE_INDEX, numTopic, topicList10Hz, enableTimestamp, freq);
  if (!(pkgStatus))
  {
    return pkgStatus;
  }

  // Start listening to the telemetry data
  subscribeStatus =
    vehicle->subscribe->startPackage(DEFAULT_PACKAGE_INDEX, responseTimeout);
  if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
  {
    ACK::getErrorCodeMessage(subscribeStatus, __func__);
    // Cleanup
    ACK::ErrorCode ack =
      vehicle->subscribe->removePackage(DEFAULT_PACKAGE_INDEX, responseTimeout);
    if (ACK::getError(ack))
    {
      std::cout << "Error unsubscribing; please restart the drone/FC to get "
                   "back to a clean state.\n";
    }
    return false;
  }
  return true;
}

bool
teardownSubscription(DJI::OSDK::Vehicle* vehicle, const int pkgIndex,
                     int responseTimeout)
{
  ACK::ErrorCode ack =
    vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
  if (ACK::getError(ack))
  {
    std::cout << "Error unsubscribing; please restart the drone/FC to get back "
                 "to a clean state.\n";
    return false;
  }
  return true;
}
//================================================================================
bool
runWaypointMission(Vehicle* vehicle, uint8_t numWaypoints, int responseTimeout)
{
  if (vehicle->getFwVersion() != Version::M100_31)
  {
    if (!setUpSubscription(vehicle, responseTimeout))
    {
      std::cout << "Failed to set up Subscription!" << std::endl;
      return false;
    }
    sleep(1);
  }

  // Waypoint Mission : Initialization
  		  
  Telemetry::GlobalPosition HomePosition;
  HomePosition = vehicle->broadcast->getGlobalPosition();

  WayPointInitSettings fdata;
  setWaypointInitDefaults(&fdata);

  fdata.indexNumber =
    numWaypoints + 1; // We add 1 to get the aircarft back to the start.

  float64_t increment = 0.000001;
  float32_t start_alt = HomePosition.altitude;

  ACK::ErrorCode initAck = vehicle->missionManager->init(
    DJI_MISSION_TYPE::WAYPOINT, responseTimeout, &fdata);
  if (ACK::getError(initAck))
  {
    ACK::getErrorCodeMessage(initAck, __func__);
  }

  vehicle->missionManager->printInfo();
  std::cout << "Initializing Waypoint Mission..\n";

  // Waypoint Mission: Create Waypoints
  std::vector<WayPointSettings> generatedWaypts =
    createWaypoints(vehicle, numWaypoints, increment, start_alt);
  std::cout << "Creating Waypoints..\n";

  // Waypoint Mission: Upload the waypoints
  uploadWaypoints(vehicle, generatedWaypts, responseTimeout);
  std::cout << "Uploading Waypoints..\n";

  // Waypoint Mission: Start
  ACK::ErrorCode startAck =
    vehicle->missionManager->wpMission->start(responseTimeout);
  if (ACK::getError(startAck))
  {
    ACK::getErrorCodeMessage(initAck, __func__);
  }
  else
  {
    std::cout << "Starting Waypoint Mission.\n";
  }

  // Cleanup before return. The mission isn't done yet, but it doesn't need any
  // more input from our side.
  if (vehicle->getFwVersion() != Version::M100_31)
  {
    return teardownSubscription(vehicle, DEFAULT_PACKAGE_INDEX,
                                responseTimeout);
  }

  return true;
}

void
setWaypointDefaults(WayPointSettings* wp)
{
  wp->damping         = 0;
  wp->yaw             = 0;
  wp->gimbalPitch     = 0;
  wp->turnMode        = 0;
  wp->hasAction       = 0;
  wp->actionTimeLimit = 100;
  wp->actionNumber    = 0;
  wp->actionRepeat    = 0;
  for (int i = 0; i < 16; ++i)
  {
    wp->commandList[i]      = 0;
    wp->commandParameter[i] = 0;
  }
}

void
setWaypointInitDefaults(WayPointInitSettings* fdata)
{
  fdata->maxVelocity    = 10;
  fdata->idleVelocity   = 5;
  fdata->finishAction   = 0;
  // 0 : do nothing after mission
  // 1 : Go home
  // 2 : Landing after mission
  fdata->executiveTimes = 1;
  fdata->yawMode        = 0;
  fdata->traceMode      = 0;
  fdata->RCLostAction   = 1;
  fdata->gimbalPitch    = 0;
  fdata->latitude       = 0;
  fdata->longitude      = 0;
  fdata->altitude       = 0;
}

std::vector<DJI::OSDK::WayPointSettings>
createWaypoints(DJI::OSDK::Vehicle* vehicle, int numWaypoints,
                float64_t distanceIncrement, float32_t start_alt)
{
  // Create Start Waypoint
  WayPointSettings start_wp;
  setWaypointDefaults(&start_wp);

  // Global position retrieved via subscription
  Telemetry::TypeMap<TOPIC_GPS_FUSED>::type subscribeGPosition;
  // Global position retrieved via broadcast
  Telemetry::GlobalPosition broadcastGPosition;

  if (vehicle->getFwVersion() != Version::M100_31)
  {
    subscribeGPosition = vehicle->subscribe->getValue<TOPIC_GPS_FUSED>();
    start_wp.latitude  = subscribeGPosition.latitude;
    start_wp.longitude = subscribeGPosition.longitude;
    start_wp.altitude  = start_alt;
    printf("Starting Waypoint created at (LLA): %f \t%f \t%f\n",
           subscribeGPosition.latitude, subscribeGPosition.longitude,
           start_alt);
  }
  else
  {
    broadcastGPosition = vehicle->broadcast->getGlobalPosition();
    start_wp.latitude  = broadcastGPosition.latitude;
    start_wp.longitude = broadcastGPosition.longitude;
    start_wp.altitude  = start_alt;
    printf("Starting Waypoint created at (LLA): %f \t%f \t%f\n",
           broadcastGPosition.latitude, broadcastGPosition.longitude,
           start_alt);
  }

  std::vector<DJI::OSDK::WayPointSettings> wpVector =
    generateWaypointsPolygon(&start_wp, distanceIncrement, numWaypoints);
  return wpVector;
}

std::vector<DJI::OSDK::WayPointSettings>
generateWaypointsPolygon(WayPointSettings* start_data, float64_t increment,
                         int num_wp)
{

  // Let's create a vector to store our waypoints in.
  std::vector<DJI::OSDK::WayPointSettings> wp_list;

  // Some calculation for the polygon
  float64_t extAngle = 2 * M_PI / num_wp;

  // First waypoint
  start_data->index = 0;
  wp_list.push_back(*start_data);

  // Iterative algorithm
  for (int i = 1; i < num_wp; i++)
  {
    WayPointSettings  wp;
    WayPointSettings* prevWp = &wp_list[i - 1];
    setWaypointDefaults(&wp);
    wp.index     = i;
    wp.latitude  = (prevWp->latitude + (increment * cos(i * extAngle)));
    wp.longitude = (prevWp->longitude + (increment * sin(i * extAngle)));
    wp.altitude  = (prevWp->altitude);
    wp_list.push_back(wp);
  }

  // Come back home
  start_data->index = num_wp;
  wp_list.push_back(*start_data);

  return wp_list;
}

void
uploadWaypoints(Vehicle*                                  vehicle,
                std::vector<DJI::OSDK::WayPointSettings>& wp_list,
                int                                       responseTimeout)
{
  for (std::vector<WayPointSettings>::iterator wp = wp_list.begin();
       wp != wp_list.end(); ++wp)
  {
    printf("Target Waypoint created at (LLA): %f \t%f \t%f\n ", wp->latitude,
           wp->longitude, wp->altitude);
    ACK::WayPointIndex wpDataACK =
      vehicle->missionManager->wpMission->uploadIndexData(&(*wp),
                                                          responseTimeout);

    ACK::getErrorCodeMessage(wpDataACK.ack, __func__);
  }
}
//==================================================================================
bool
runHotpointMission(Vehicle* vehicle, int initialRadius, int responseTimeout)
{
  if (vehicle->getFwVersion() != Version::M100_31)
  {
    if (!setUpSubscription(vehicle, responseTimeout))
    {
      std::cout << "Failed to set up Subscription!" << std::endl;
      return false;
    }
    sleep(1);
  }

  // Global position retrieved via subscription
  Telemetry::TypeMap<TOPIC_GPS_FUSED>::type subscribeGPosition;
  // Global position retrieved via broadcast
  Telemetry::GlobalPosition broadcastGPosition;

  // Hotpoint Mission Initialize
  vehicle->missionManager->init(DJI_MISSION_TYPE::HOTPOINT, responseTimeout,
                                NULL);
  vehicle->missionManager->printInfo();

  if (vehicle->getFwVersion() != Version::M100_31)
  {
    subscribeGPosition = vehicle->subscribe->getValue<TOPIC_GPS_FUSED>();
    vehicle->missionManager->hpMission->setHotPoint(
      subscribeGPosition.longitude, subscribeGPosition.latitude, initialRadius);
  }
  else
  {
    broadcastGPosition = vehicle->broadcast->getGlobalPosition();
    vehicle->missionManager->hpMission->setHotPoint(
      broadcastGPosition.longitude, broadcastGPosition.latitude, initialRadius);
  }

  // Takeoff
  ACK::ErrorCode takeoffAck = vehicle->control->takeoff(responseTimeout);
  if (ACK::getError(takeoffAck))
  {
    ACK::getErrorCodeMessage(takeoffAck, __func__);

    if (vehicle->getFwVersion() != Version::M100_31)
    {
      teardownSubscription(vehicle, DEFAULT_PACKAGE_INDEX, responseTimeout);
    }
    return false;
  }
  else
  {
    sleep(15);
  }

  // Start
  std::cout << "Start with default rotation rate: 15 deg/s" << std::endl;
  ACK::ErrorCode startAck =
    vehicle->missionManager->hpMission->start(responseTimeout);
  if (ACK::getError(startAck))
  {
    ACK::getErrorCodeMessage(startAck, __func__);
    if (vehicle->getFwVersion() != Version::M100_31)
    {
      teardownSubscription(vehicle, DEFAULT_PACKAGE_INDEX, responseTimeout);
    }
    return false;
  }
  sleep(20);

  // Pause
  std::cout << "Pause for 5s" << std::endl;
  ACK::ErrorCode pauseAck =
    vehicle->missionManager->hpMission->pause(responseTimeout);
  if (ACK::getError(pauseAck))
  {
    ACK::getErrorCodeMessage(pauseAck, __func__);
  }
  sleep(5);

  // Resume
  std::cout << "Resume" << std::endl;
  ACK::ErrorCode resumeAck =
    vehicle->missionManager->hpMission->resume(responseTimeout);
  if (ACK::getError(resumeAck))
  {
    ACK::getErrorCodeMessage(resumeAck, __func__);
  }
  sleep(10);

  // Update radius, no ACK
  std::cout << "Update radius to 1.5x: new radius = " << 1.5 * initialRadius
            << std::endl;
  vehicle->missionManager->hpMission->updateRadius(1.5 * initialRadius);
  sleep(10);

  // Update velocity (yawRate), no ACK
  std::cout << "Update hotpoint rotation rate: new rate = 5 deg/s" << std::endl;
  HotpointMission::YawRate yawRateStruct;
  yawRateStruct.clockwise = 1;
  yawRateStruct.yawRate   = 5;
  vehicle->missionManager->hpMission->updateYawRate(yawRateStruct);
  sleep(10);

  // Stop
  std::cout << "Stop" << std::endl;
  ACK::ErrorCode stopAck =
    vehicle->missionManager->hpMission->stop(responseTimeout);

  std::cout << "land" << std::endl;
  ACK::ErrorCode landAck = vehicle->control->land(responseTimeout);
  if (ACK::getError(landAck))
  {
    ACK::getErrorCodeMessage(landAck, __func__);
  }
  else
  {
    // No error. Wait for a few seconds to land
    sleep(10);
  }

  // Clean up
  ACK::getErrorCodeMessage(startAck, __func__);
  if (vehicle->getFwVersion() != Version::M100_31)
  {
    teardownSubscription(vehicle, DEFAULT_PACKAGE_INDEX, responseTimeout);
  }

  return true;
}
