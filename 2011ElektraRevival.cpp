#include "WPILib.h"
#include "victor.h"
#include "solenoid.h"
#include "gyro.h"
#include "Relay.h"
#include "DigitalInput.h"
#include "Compressor.h"

/**
 * This is a demo program showing the use of the RobotBase class.
 * The IterativeRobot class is the base of a robot application that will automatically call your
 * Periodic methods for each packet based on the mode.
 */ 
class RobotDemo : public IterativeRobot
{
	Joystick *stick; // only joystick
	Victor *FR, *FL, *BR, *BL;
	float myY, myZ, RThrottle, LThrottle, myAngle;
	Solenoid *wheelsa, *wheelsb, *arma, *armb;
 	Joystick *stick2; // only joystick two
	Gyro *gyro1;
//	AxisCamera *cam;
	int camBrightness;
	DigitalOutput *camlight;
	Ultrasonic *sonar1;
		float sOut, sP, sonarRange;
	
	
	
	Relay *spike;
	DigitalInput *banner;
	//bool mybanner;
	
public:
	RobotDemo()

		
	{

		this->SetPeriod(0); 	//Set update period to sync with robot control packets (20ms nominal)
	}
	
/**785
 * Robot-wide initialization code should go here.
 * 
 * Use this method for default Robot-wide initialization which will
 * be called when the robot is first powered on.  It will be called exactly 1 time.
 */
void RobotDemo::RobotInit() {
	
	//Initializing Everything
	
	stick = new Joystick(1);
	FR = new Victor(1);
	FL = new Victor(3);
	BR = new Victor(2);
	BL = new Victor(4);
	wheelsa = new Solenoid(1);
	wheelsb = new Solenoid(2);
	arma = new Solenoid(3);
	armb = new Solenoid(4);
	sonar1 = new Ultrasonic(1,1);
	
//	cam = new AxisCamera();
	
	stick2 = new Joystick (2);
	
	gyro1 = new Gyro(1);
	
	spike = new Relay(1);
	banner = new DigitalInput(1, 1);
	
	
}

/**
 * Initialization code for disabled mode should go here.
 * 
 * Use this method for initialization code which will be called each time
 * the robot enters disabled mode. 
 */
void RobotDemo::DisabledInit() {
}

/**
 * Periodic code for disabled mode should go here.
 * 
 * Use this method for code which will be called periodically at a regular
 * rate while the robot is in disabled mode.
 */
void RobotDemo::DisabledPeriodic() {
}

/**
 * Initialization code for autonomous mode should go here.
 * 
 * Use this method for initialization code which will be called each time
 * the robot enters autonomous mode.
 */
void RobotDemo::AutonomousInit() {	
}

/**
 * Periodic code for autonomous mode should go here.
 *
 * Use this method for code which will be called periodically at a regular
 * rate while the robot is in autonomous mode.
 */
void RobotDemo::AutonomousPeriodic() {
	if (banner->Get()== 0){
		spike->Set(Relay::kOn);
	} else{
		spike->Set(Relay::kOff);
	}
}

/**
 * Initialization code for teleop mode should go here.
 * 
 * Use this method for initialization code which will be called each time
 * the robot enters teleop mode.
 */
void RobotDemo::TeleopInit() {
	
	//Initializing Everything
//	camlight->Set(1);
}

/**
 * Periodic code for teleop mode should go here.
 *
 * Use this method for code which will be called periodically at a regular
 * rate while the robot is in teleop mode.
 */
void RobotDemo::TeleopPeriodic() {
	
/*		sonarRange = sonar1->GetRangeInches(); 
		printf("%f \n", sonarRange);
		sOut = sP * sonarRange;
		FR->SetSpeed(sOut);
		FL->SetSpeed(-sOut);
		BL->SetSpeed(-sOut);
		BR->SetSpeed(sOut);*/
	
	/*myAngle = gyro1->GetAngle();
	printf("%f",myAngle);
	
	FR->Set(-myAngle/360.0);
	BR->Set(-myAngle/360.0);
	FL->Set(myAngle/360.0);
	BL->Set(myAngle/360.0);*/
	
	myY=stick->GetY();
	myZ=stick->GetZ();	
	if (myY < .05 && myY>-.05){
		myY=0;	
	} 
	if (myZ< .05 && myZ>-0.05){
		myZ=0;
	} 
		LThrottle=myY-myZ;
		RThrottle=myY+myZ;
		FR->Set(-RThrottle);
		BR->Set(-RThrottle);
		FL->Set(LThrottle);
		BL->Set(LThrottle);

	//Solenoids
	if (stick->GetRawButton(6)){
			wheelsa->Set(false);
			wheelsb->Set(true);
	}
	if (stick->GetRawButton(8)){
			wheelsb->Set(false);
			wheelsa->Set(true);
	}
	//Arm 
	if (stick->GetRawButton(5)){
			arma->Set(true);
			armb->Set(false);
	}
	if (stick->GetRawButton(7)){
			arma->Set(false);
			armb->Set(true);
	}
	
	//Camera Code
//	camBrightness = cam->GetBrightness();
//	printf("%i \n", camBrightness);
}

/**
 * Initialization code for test mode should go here.
 * 
 * Use this method for initialization code which will be called each time
 * the robot enters test mode.
 */
void RobotDemo::TestInit() {
}

/**
 * Periodic code for test mode should go here.
 *
 * Use this method for code which will be called periodically at a regular
 * rate while the robot is in test mode.
 */
void RobotDemo::TestPeriodic() {
}

};

START_ROBOT_CLASS(RobotDemo);


/* The Fate of Electra: This code has worked for Electra. The gripper, the drive base, the banner system was all operable. */
