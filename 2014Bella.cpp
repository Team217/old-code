#include "WPILib.h"
#include "IOStream.h"
#include "Math.h"
#include <cmath>
//Received from EVAN @ 3/13/14
//Comp bot code
class RobotDemo : public IterativeRobot
{
	// Teleop Variables
	float speed, turn, strafe, leftout, rightout, FeederSpeed, fRout, rRout, fLout, rLout;
	bool WheelState, OnMechanums, ManualControl;
	//Toilet Variables
	float ToiletSpeed, ToiletPos, GyroAngle, ToiletError, TPout, TIout, ToiletTar;
	 //Auton Variables
	float AutonError, AutonPOut, AutonIOut, AutonLSpeed, AutonRSpeed, AutonPosR, AutonPosL,TarInFt, TrxnSensitivity, OmniSpeed, EncoderAvg, AutonSensitivity;
	bool HasFired, SecondRun, ThirdRun ,SecondFired, ThirdFired, BallLimitPressed, ChangingValues, ArmState;
	int RunCount;
	//Value Holders for non DB encoders
	float Tvalue, fRvalue, fLvalue;
	static const float sensitivity = .12; //Sets deadband sensitivity
	
	float OMNI_TAR; //Feet
	float TRXN_TAR;
	float THIRD_TAR;
	float SHOT_TAR;
	//Toilet Encoder Pre-sets
	static const double AUTONSHOT = 320;
	static const double ONEBALLSHOT = 270;
	
	float shotTarget;
	float AutonAve;
	
	static const float OmniTcksPerIn = 128/3.141592653589;
	static const float TrxnTcksPerIn = 288/3.141592653589;
	
	static const double PRESET_1 = 240.25; //Normal inflation - WAS 280
	static const double PRESET_2 = 280.5; //Over inflation - WAS 276
	static const double PRESET_3 = 305; //under inflation - was 200
	static const double PRESET_4 = 200;
	float AutonTarget;
	
	//Toilet PID
	static const float ToiletKP = .04; // was 0.02
	static const float ToiletKI = .015; // was 0.0075
	
	//Drive Base PID
	static const float AutonKP = .00035;// .000112; //???? Probably should change
	static const float AutonKI = .000012; // .0000112;
	

	
	enum AutonStateType{
		wait,
		autonShoot,
		stop,
		pickUp,
		oneBall,
		recharge,
		suckIn
	}AutonState;
	
	enum AutonBallType{
		OneBallAuton,
		TwoBallAuton,
		ThreeBallAuton
	}AutonType;
	
	//Sensors & Motors
	//	Analog Channel
	AnalogChannel *DriveSwitch;
	
	//	Encoders
	Encoder *fRe, *fLe, *toiletE;
	
	//	Gyros	
	Gyro *g1;
	
	//	Joysticks
	Joystick *driver, *oper;
	//	Motors
	Victor *rR1, *rR2, *fR1, *fR2, *rL1, *rL2, *fL1, *fL2, *Shooter, *Feeder, *Toilet;
	
	//	Digital Inputs
	//		Limit Switches
	DigitalInput *seatUp, *seatDown, *FeederSwitch, *AutonSwitch;
	//		Hall Effect
	DigitalInput *halle;
	//		Auton button
	DigitalInput *ballHeld;
	
	//	Hall Effect Variables
	bool halleflag, HEVoltage;
		
	//Kicker Bools
	int MagnetInWay;
		
	//Compressor Variables
	Compressor *comp;
	
	//Solenoids
	//	Arm
	DoubleSolenoid *FeederArm;
	//	Drivebase
	Solenoid *fRs, *rRs, *fLs, *rLs;
	

	
	//Misc Variables
	Ultrasonic *sonar;
	float sonarValue;
		
	//Timers
	Timer *autonKick, *autonFeed, *firstShot, *holdup, *myClock1, *myClock2, *myClock3;
	
	//Macro Move Keys - J.L.
	enum MacroKey{
		kNormal, kLRollDodge, kRRollDodge, kLSpin, kRSpin
	} macro;
	
	
public:
	RobotDemo()
	{		
		/*
		 * Joystick init
		 */
		driver = new Joystick(1);
		oper = new Joystick(2);
		
		
		/*
		 * Victor init
		 */
		fR1 = new Victor(1,2); //front right
		//fR2 = new Victor(1,1); //front right mini
		
		rR1 = new Victor(2,2); //rear right
		//rR2 = new Victor(2,1); //rear right mini
		
		fL1 = new Victor(1,4); //front left
		//fL2 = new Victor(1,3); //front left mini
		
		rL1 = new Victor(2,4); //rear left
		//rL2 = new Victor(2,3); //rear left mini
		
		Shooter = new  Victor(1,5);
		Feeder = new Victor(1,6);
		Toilet = new Victor(2,5);
		
		
		
		/*
		 * Encoder init
		 */
		
		fRe = new Encoder (1,4,1,5, true, CounterBase::k4X);
		fLe = new Encoder (1,2,1,3, false, CounterBase::k4X);
		toiletE = new Encoder(1,7,1,6,false,CounterBase::k4X);//false on comp true on practice
		
		
		
		/*
		 * Pneumatic system init
		 
		fRs = new DoubleSolenoid(1,2); //Front right
		
		rRs = new DoubleSolenoid(3,4); //Rear right
		
		fLs = new DoubleSolenoid(5,6); //Front left
		
		rLs = new DoubleSolenoid(7,8); //Rear left
		
		*/
		
		fRs = new Solenoid(1,2); //front right
		fLs = new Solenoid(1,1); //front left
		rRs = new Solenoid(1,4); //rear right
		rLs = new Solenoid(1,3); //rear left
		
		FeederArm = new DoubleSolenoid(5,6);
		
		comp = new Compressor(1,13,1,1); // (1,1,1,1) on practice, (1,13,1,1) on comp
		
		
		//Hall Effect
		halle = new DigitalInput (1,14);
		HEVoltage = 0; 
			
		//Limit Switches
		seatUp = new DigitalInput(1,8);
		seatDown = new DigitalInput(1,9);
		ballHeld = new DigitalInput(1,12);
		
		//FeederSwitch = new DigitalInput()
		AutonSwitch = new DigitalInput(2,2);
		g1 = new Gyro(1,1);
		
		//Timers
		holdup = new Timer();
		myClock1 = new Timer();
		myClock2 = new Timer();
		myClock3 = new Timer();
	
		//Update period
		this->SetPeriod(0); 	//Set update period to sync with robot control packets (20ms nominal)
	}
void RobotDemo::RobotInit() {
	
}
void RobotDemo::DisabledInit() {
	//Stop Compressor
	comp->Stop();
	macro = kNormal;
	
}
void RobotDemo::DisabledPeriodic() {
}
void RobotDemo::AutonomousInit() {

	Feeder->SetSpeed(.25);
	
	//Start comp + encoders
	comp->Start();
	toiletE->Start();
	fLe->Start();
	fRe->Start();
	
	//Reset encoders
	toiletE->Reset();
	fLe->Reset();
	fRe->Reset();
	g1->Reset();
	
	myClock1->Stop();
	myClock1->Reset();
	
	myClock2->Stop();
	myClock2->Reset();	
	AutonTarget = TicksOnOmni(15.17);
	//Txn Wheels
	WheelState = true;
	OmniWheelState(WheelState);
	
	//Wait for traction wheels to hit the ground
	Wait(.25);
	
	//Haven't fired
	HasFired = false;
	//HasFired2 = false;
	
	//Set Toilet target to standard shot
	ToiletTar = PRESET_1;
	
	//Robot has not moved yet - Aaron's shitty auton
	//hasDriven = false;
	AutonState = wait;
	if(!AutonSwitch->Get()){
		AutonType = TwoBallAuton;
	}else AutonType = OneBallAuton;
}
void RobotDemo::AutonomousPeriodic() {
	//Run Toilet PID
	GyroAngle = g1->GetAngle();
	Tvalue = toiletE->GetRaw();
	printf("%f\n",GyroAngle);
	ToiletPID();
	AutonPID();
	//GyroAdjust();


	//Set toilet speed from PID system
	Toilet->Set(ToiletSpeed);
if(AutonType == OneBallAuton){
switch(AutonState){
	case wait:
		if(ToiletError <= 6 && ToiletError >= -6){
			Wait(0.09);
			AutonState = autonShoot;
		}
		break;	
	case autonShoot:
		SetRSpeed(AutonRSpeed);
		SetLSpeed(AutonLSpeed);
		MagnetInWay = !halle->Get();	
		//Where's magnet?
		if(!MagnetInWay)
			//Not in the way...
			Shooter->Set(-1);
		else{
			//In the way...
			Shooter->Set(0);
		}
		if(AutonError < 5 && !HasFired){
			Shooter->Set(-1);
			Wait(.2);
			HasFired = true;
			AutonState = stop;
		}
		break;
		
	case stop:
		SetRSpeed(0);
		SetLSpeed(0);
		Feeder->Set(0);
		Shooter->Set(0);
		//ToiletReset();
	default:
		break;
	}
}
else if(AutonType == TwoBallAuton){
	
switch(AutonState){
	case wait:
		ToiletTar = PRESET_2;
		if(ToiletError <= 5 && ToiletError >= -5){
			Wait(0.09);
			AutonState = suckIn;
		}
		break;
	case suckIn:
		SolenoidState(FeederArm, true);
		if(ballHeld->Get() == true){
			Feeder->SetSpeed(.65);//was .75
		}else{
			Feeder->SetSpeed(0);
			AutonState = oneBall;
		}
		//if(myClock3->Get() >= 0.1)
			//AutonState = oneBall;
		break;
	case oneBall:
		SetRSpeed(AutonRSpeed);
		SetLSpeed(AutonLSpeed);
		
		//Get magnet position
		MagnetInWay = !halle->Get();	
		//Where's magnet?
		if(!MagnetInWay)
			//Not in the way...
			Shooter->Set(-1);
		else
			//In the way...
			Shooter->Set(0);
				
		//Auton PID			

		//If drivebase within error range, stop
/*		if(AutonError < 300 && AutonError > -300){
				AutonLSpeed = 0;
				AutonRSpeed = 0;
		}*/
		
		//Set drive base speed

		
		//If drive speed is zero and we haven't fired, FIRE
		if(AutonAve > shotTarget && !HasFired){
			//Fire
			Shooter->Set(-1);
			ToiletTar = PRESET_1;
			myClock2->Start();
			
			//Wait to pass the shooting point
			Wait(.2);
			
			//Switch the var
			HasFired = true;

			AutonState = recharge;
			
			//ToiletTar = PRESET_1;
		}/*else if(!HasFired2 && BallLimitPressed){
			if(MagnetInWay){
			SolenoidState(FeederArm, true);
			Wait(3.0);
			Shooter->Set(-1);
			Wait(.25);
			BallLimitPressed = false;
			HasFired2 = true;
			}
		}*/
		break;
	
	case recharge:
		SetRSpeed(AutonRSpeed);
		SetLSpeed(AutonLSpeed);
		MagnetInWay = !halle->Get();	
		//Where's magnet?
		if(!MagnetInWay)
			//Not in the way...
			Shooter->Set(-1);
		else{
			//In the way...
			Shooter->Set(0);
		}
		if(myClock2->Get() >= 2){
			SolenoidState(FeederArm, false);
			myClock1->Start();
			AutonState = pickUp;
		}
		break;
	case pickUp:
		SetRSpeed(AutonRSpeed);
		SetLSpeed(AutonLSpeed);
		Feeder->SetSpeed(0.75);
		SolenoidState(FeederArm, false);

		//Get magnet position
		MagnetInWay = !halle->Get();	
		//Where's magnet?
		if(!MagnetInWay)
			//Not in the way...
			Shooter->Set(-1);
		else
			//In the way...
			Shooter->Set(0);
		
		if(myClock1->Get() >= 3.5 && AutonLSpeed == 0 && AutonRSpeed == 0){
			//Fire
			Shooter->Set(-1);
			Wait(.25);
			AutonState = stop;
		}
		break;
		
	case stop:
		SetRSpeed(0);
		SetLSpeed(0);
		Feeder->Set(0);
		Shooter->Set(0);
		//ToiletReset();
	default:
		break;
	}
}
}
//Auton Drivebase PID loop - Jake Laird
void AutonPID(){
	AutonPosL = fLe->GetRaw();
	AutonPosR = fRe->GetRaw();
	
	shotTarget = 8.772; //ADDED 1 REV (1/4 OF 16FT = 1955.696 TCKS
	
	AutonAve = (AutonPosR+AutonPosL)/2.00;
	
	AutonError = AutonTarget - AutonAve;
	
	AutonPOut = AutonError*AutonKP;
	
	AutonIOut = AutonError*AutonKI;
	
	AutonRSpeed = (PIDDeadband(AutonPOut+AutonIOut, .35));
	AutonLSpeed = -(PIDDeadband(AutonPOut+AutonIOut, .35));
	
	if(AutonRSpeed >= 0.55){
		AutonRSpeed = 0.55;
	}
	if(AutonRSpeed <= -.55){
		AutonRSpeed = -0.55;
	}
	if(AutonLSpeed >= 0.55){
		AutonLSpeed = 0.55;
	}
	if(AutonLSpeed <= -.55){
		AutonLSpeed = -0.55;
	}
	
	if(AutonAve <= 200){
		if(AutonRSpeed >= 0.450){
			AutonRSpeed = 0.450;
		}
		if(AutonRSpeed <= -450){
			AutonRSpeed = -0.450;
		}
		if(AutonLSpeed >= 0.450){
			AutonLSpeed = 0.450;
		}
		if(AutonLSpeed <= -.450){
			AutonLSpeed = -0.450;
		}
	}
	
}



void GyroAdjust(){
	if(GyroAngle >= 1){
		AutonLSpeed = 0;//AutonLSpeed - GyroAngle*.1;
	}
	if(GyroAngle <= -1){
		AutonRSpeed = 0;//AutonRSpeed + GyroAngle*.1;
	}
}


float TicksOnTrxn(float ft){
//	return (ft*((257.83100781*12)/3));
	return ((ft * 12)*TrxnTcksPerIn);
}
float TicksOnOmni(float ft){
	//return (ft*1375);
	return ((ft * 12)*OmniTcksPerIn);
}

void RobotDemo::TeleopInit() {
	comp->Start();
	toiletE->Start();
	fLe->Start();
	fRe->Start();
	WheelState = true;
	OmniWheelState(WheelState);
	ManualControl = false;
	OnMechanums = false; //For now

}

void RobotDemo::TeleopPeriodic() {	
	GyroAngle = g1->GetAngle();
	Tvalue = toiletE->GetRaw();
	//printf("Toilet: %f \n",Tvalue);
	fLvalue = fLe->GetRaw();
	//printf("Left: %f ", fLvalue);
	fRvalue = fRe->GetRaw();
	//printf("Right: %f ", fRvalue);
	//Gyro Angle Code
	GyroReset();
	
	//printf("%f \n", GyroAngle);
	
	//printf("%f \n", toiletE->GetRaw());
	
	if(OnMechanums == true){
		// need a switch for mecanum wheels
		speed = Deadband(driver, driver->GetY(), sensitivity);
		strafe = Deadband(driver, driver->GetX(), sensitivity);
		turn = Deadband(driver, driver->GetThrottle(), sensitivity);
		
		fRout = -(speed-strafe-turn);
		rRout = -(speed+strafe-turn);					
		fLout = speed+strafe+turn;
		rLout = speed-strafe+turn;
					
		rightout = -((Deadband(driver, speed, sensitivity))+Deadband(driver, turn, sensitivity));
		leftout = (Deadband(driver, speed, sensitivity)-(Deadband(driver, turn, sensitivity)));
		
		MechanumDrive();
	}
	else{
	
	speed = driver->GetY();
	turn = driver->GetThrottle();
	rightout = -((Deadband(driver, speed, sensitivity))+Deadband(driver, turn, sensitivity));
	leftout = (Deadband(driver, speed, sensitivity)-(Deadband(driver, turn, sensitivity)));
	
	/*
	 * MACRO BUTTON LOGIC FOR DRIVING
	 */
/*	if(WheelState == true){
		OmniMacros();
		switch(macro){
			case kLRollDodge:
			case kRRollDodge: 
				RollDodge();
				break;
			case kLSpin:
			case kRSpin:
				FastSpin();
				break;
				
			default:NormalDrive();
			break;
		}
		
	}else NormalDrive();
				*/
	NormalDrive();
		}
	
	Feed();
	Shoot();

	//CompressorLogic();
	}



/*
 * DRIVER FUNCTIONS START HERE
 */

/*
 *Compressor logic to save battery for onboard compressor *NEEDS WORK*
 *
 *Should stop compressor until pressure is below 80
 */
void CompressorLogic(){
	if (comp->GetPressureSwitchValue()<80)
			comp->Start();
	else if(comp->GetPressureSwitchValue()>115)
				comp->Stop();
			
}



/*
 * Macro buttons for omni wheels
 */
void OmniMacros(){
	if(driver->GetRawButton(1))
					macro = kLSpin;
		else if(driver->GetRawButton(2))
					macro = kLRollDodge;
			else if(driver->GetRawButton(3))
					macro = kRRollDodge;/* Comment out if com lost*/
			else if(driver->GetRawButton(4))
					macro = kRSpin;
			else macro = kNormal;
}



/*
 * The deadband settings
 */
float Deadband(Joystick *driver, float axis, float s){
			if (axis<-s || axis>s) return axis;
			else return 0;
		}



/*
 * Basic driving when no maneuvers are called on
 * Button 6: omnis
 * Button 8: traction
 */
void NormalDrive(){
	if (driver->GetRawButton(5)){
		rightout = 0.5;
		leftout = -0.5;
	} 
	SetRSpeed(rightout);
	SetLSpeed(leftout);
			if (driver->GetRawButton(6)){
				WheelState = true;
				OmniWheelState(WheelState);
			}else if(driver->GetRawButton(8)){
				WheelState = false;
				OmniWheelState(WheelState);
		}
}



void MechanumDrive(){
	if (driver->GetRawButton(6)){
			WheelState = true;
			OmniWheelState(WheelState);
		} else if(driver->GetRawButton(8)){
			WheelState = false;
			OmniWheelState(WheelState);
		}
		
	if (WheelState == true){
			fL1->Set(fLout);
			//fL2->Set(fLout);
			rL1->Set(rLout);
			//rL2->Set(rLout);
					
			fR1->Set(fRout);
			//fR2->Set(fRout);
			rR1->Set(rRout);
			//rR2->Set(rRout);
		}else{
			SetRSpeed(rightout);
			SetLSpeed(leftout);
		}
}



/*
 * Performs an extremely fast turn while the button is held down.
 */
void FastSpin(){
	if(macro == kLSpin){
	while(driver->GetRawButton(1)){
		//SolenoidState(fLs, false);
		fLs->Set(true);
		SetRSpeed(1);
		SetLSpeed(1);
		}
	}
	else if(macro == kRSpin){
		while(driver->GetRawButton(4)){
		//SolenoidState(fRs, false);
		fRs->Set(true);
		SetRSpeed(-1);
		SetLSpeed(-1);	
		}
	}
	OmniWheelState(true);
}



/*
 * Roll dodge- sets wheel thrown down as the pivot and spins around it
 */
void RollDodge(){
	if(macro == kRRollDodge)
		while(driver->GetRawButton(3)){
			//SolenoidState(fLs, false);
			fLs->Set(true);
			SetRSpeed(1);
			rL1->Set(.5);
			rL2->Set(.5);
			
		}
	else if(macro == kLRollDodge){
		while(driver->GetRawButton(2)){
			//SolenoidState(fRs, false);
			fRs->Set(true);
			SetLSpeed(-1);
			rR1->Set(-.5);
			rR2->Set(-.5);
		}
	}
	OmniWheelState(true);
}
	


/*
 * Sets the chosen solenoid to Forward State if true, or Reverse State if false
 */
void SolenoidState(DoubleSolenoid *s, bool b){
	s->Set(b ? DoubleSolenoid::kForward : DoubleSolenoid::kReverse);

}



/*
 * Sets whole drive base on omnis if true, traction if false.
 */
void OmniWheelState(bool b){
	/*
	SolenoidState(fRs, b);
	SolenoidState(rRs, b);
	SolenoidState(fLs, b);
	SolenoidState(rLs, b);
	*/
	b = !b;
	fRs->Set(b);
	rRs->Set(b);
	fLs->Set(b);
	rLs->Set(b); 	 
	 
}



/*
 * Functions to set left and right side motor throttle at once.
 */
void SetRSpeed(float v){
	rR1->Set(v);
	//rR2->Set(v);
	fR1->Set(v);
	//fR2->Set(v);

}
void SetLSpeed(float v){
	rL1->Set(v);
	//rL2->Set(v);
	fL1->Set(v);
	//fL2->Set(v);
}



/*
 * DRIVER FUNCTIONS END HERE
 *
 * OPERATOR FUNCTIONS START HERE
 */


/*
 * Feeding/Puking
 */
void Feed(){
	
	if(oper->GetRawButton(9)){
						toiletE->Reset();
						fRe->Reset();
						fLe->Reset();
					}
	if(oper->GetRawButton(7))
		SolenoidState(FeederArm, true); 
	else if(oper->GetRawButton(5))
		SolenoidState(FeederArm, false);
	
	FeederSpeed = Deadband(oper, -(oper->GetRawAxis(6)), sensitivity);
	Feeder->Set(FeederSpeed);
}



/*
 * Shooting and angle code goes here PID/camera code maybe?
 */
void Shoot(){
	if(oper->GetRawButton(10)){
					ManualControl = ~ManualControl;
				}
	
	if(ManualControl == true){
		/*
		 * MANUAL CONTROLS GO HERE...
		 * no presets
		 */
		if(oper->GetRawButton(6) && oper->GetRawButton(8))
			Shooter->Set(-1);
		else Shooter->Set(0);
		
	}else{
		
		MagnetInWay = !halle->Get();
			if(!MagnetInWay)
				Shooter->Set(-1);
				else Shooter->Set(0);
			
		if(oper->GetRawButton(6) && oper->GetRawButton(8))
		{
			Shooter->Set(-1);
		}
	}
	
	if(oper->GetRawButton(1))
		
		ToiletTar = PRESET_1;
			
	if(oper->GetRawButton(2))
				
		ToiletTar = PRESET_2;
	
	//truss shot
	if(oper->GetRawButton(4)){
		ToiletTar = PRESET_4;
	}
	//if(oper->GetRawButton(4))
		
		//crazy proximity code
		/*sonar->SetEnabled(true);
		sonarValue = (sonar->GetRangeInches()/12.0);
		printf("%f\n",sonarValue);
		sonar->SetEnabled(false);
					//change angle for ??? shot*/
		
	if(oper->GetRawButton(3))
		
		ToiletTar = PRESET_3;
	

			
	if(Deadband(oper, oper->GetY(), sensitivity) != 0){
		ToiletSpeed = Deadband(oper, oper->GetY(), .25);
		ToiletTar = toiletE->GetRaw();
	}else{
		ToiletPID();
	}
	
	ToiletLimit();
	Toilet->Set(ToiletSpeed);
/*	if(ToiletSpeed != 0)
		comp->Stop();
	else
		comp->Start();*/
	if(ToiletSpeed <= 0.25 && ToiletSpeed >= -0.25){
		comp->Start();
	}
	else
		comp->Stop();
	//	printf("%f",toiletE->GetRaw());
}



/*
 * Preset shots
 */
void GyroReset(){
	//Gyro Angle Code
	if (GyroAngle >= 360 || GyroAngle <= -360){
		g1->Reset();
	}
};

void ToiletPID(){
	ToiletPos = toiletE->GetRaw();
	ToiletError = ToiletTar - ToiletPos;
	TPout = ToiletError*ToiletKP;
	TIout = ToiletError*ToiletKI;
	ToiletSpeed = PIDDeadband(TPout+TIout, .2);
	
}



float PIDDeadband(float o, float s){
			if (o < s && o > -s)
				return 0;
			return o;
		}



/*
 * Limit switch code for toilet
 */
void ToiletLimit(){
	if (!seatUp->Get() && ToiletSpeed<=0){
				//Zero the seat motor
				ToiletSpeed = 0;
			}
	
		//	If we hit lower limit and we're trying to slide seat backwards
		if (!seatDown->Get() && ToiletSpeed>=0){
				//Zero the seat motor
				ToiletSpeed = 0;
			}
		//	If we hit upper limit and we're trying to slide seat backwards
}


/*
 * OPERATOR FUNCTIONS END HERE
 * 
 * IN PIT CODE FOR TESTING
 */

void RobotDemo::TestInit() {
	
	comp->Start();
	
	OmniWheelState(false);
		Wait(1.0);
		OmniWheelState(true);
		Wait(1.0);
		SetLSpeed(1);
		SetRSpeed(-1);
		Wait(1.0);
		SetLSpeed(-1);
		SetRSpeed(1);
		Wait(1.0);
		SetLSpeed(1);
		SetRSpeed(1);
		Wait(1.0);
		SetLSpeed(-1);
		SetRSpeed(-1);
		Wait(1.0);
		SetRSpeed(0);
		SetLSpeed(0);
		SolenoidState(FeederArm,true);
		Wait(2.0);
		SolenoidState(FeederArm,false);
		Feeder->Set(1.0);
		Wait(1.5);
		Feeder->Set(-1.0);
		Wait(1.5);
		Feeder->Set(0);
		while(seatDown->Get()){
			Toilet->Set(1);
		}
		ToiletReset();
		myClock3->Start();
		
}

void ToiletReset(){
	while(seatUp->Get()){
		Toilet->Set(-1);
	}
	Toilet->Set(0);
	toiletE->Reset();
}

void RobotDemo::TestPeriodic() {
//	if(myClock3->Get() <= 2)
//		ToiletTar = PRESET_1;
//	if(myClock3->Get() > 2 && myClock3->Get() <= 4)
//		ToiletTar = PRESET_2;
//	if(myClock3->Get() > 4 && myClock3->Get() <= 6)
//		ToiletTar = PRESET_3;
//	if(myClock3->Get() > 6 && myClock3->Get() <= 8)
//		ToiletTar = PRESET_4;
//	
//	if(myClock3->Get() < 8){
//		ToiletPID();
//		Toilet->Set(ToiletSpeed);
//	}
//	else
//		ToiletReset();
}

};

START_ROBOT_CLASS(RobotDemo);

