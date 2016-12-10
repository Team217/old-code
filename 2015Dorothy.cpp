#include "WPILib.h"

class Robot: public IterativeRobot
{
	private:
	/*DRIVE BASE*/
	SendableChooser *chooser;bool snakeOn = false;

	//translational talons
	CANTalon *fl, *fr, *bl, *br;

	//translational encoders
	Encoder *flE, *frE, *blE, *brE;

	//rotational talons
	CANTalon *flS, *frS, *blS, *brS, *currentS;

	// rotational encoders 0=fl, 1=fr, 2=bl, 3=br;
	AnalogInput *flSE, *frSE, *blSE, *brSE;

	//compressor
	Compressor *comp;bool compOn;

	//mast
	Solenoid *mastA, *mastB;

	//Teleop whether to rotate or translate
	int mix;

	// Operator button 13: for auto load from human player station
	enum
	{
		p3, open, p2, close_preset, p4
	} drop;

	//Auton to grab a sigle recycling can and drag to autozone
	enum
	{
		closes, raise2, back1, turn45prep, turn45, endOneCan, teleopprep
	} OneCanAuton;

	//Auton to grab two cans from center
	enum
	{
		grab2C = 1, drag2C = 2, up2C = 3, forksdown = 4,
	} Twocan;

	//Second attempt at a three yellow tote auton. Robot goes over cans like a frog.
	enum
	{
		prep1, up1, over1, down1, clamp2, up2, over2, down2, clamp3, up3, back, open2, finish2
	} SecondThreeTote;

	enum
	{
		visstrafeprep = 1, visstrafe = 2, donedefault = 3
	} VisionState;

	enum
	{
		clamp1, strafe1prep, strafe1, forward1prep, forward1, strafe2prep
	} OneMoreTry;
	//Extremely useful boolean so one cpp can be utilised.
	bool autonCheck = 0;bool clap = 1;

	PowerDistributionPanel *pdp;
	//Camera Vision
	NetworkTable *table;
	Image *frame;
	IMAQdxSession session;
	IMAQdxError imaqError;
	float camID = 0;

	float objx = 0;
	float objxI = 0.005;//lower this bc jerking
	float objxP = 0.01;//lower this bc jerking
	float objxPos = 160;

	float cogPIDPos = 0;
	float cogPIDTar = 0;
	float cogPIDError = 0;
	float cogPIDPout = 0, cogPIDIout = 0, cogPIDSpeed = 0;bool visionpushed = 0;

	//Teleop control variables
	float mag, angleD;

	//PID constants
	float PID_P = 0.0001;//0.0005 * .9;
	const float MAX_PID_P = 0.012 * .9;
	float PID_I = .00018;//0.0001 * .9;
	float aError;
	float PID_auton = .003;bool PID_I_CHECK = true;

	//Hopefully implemented i variables for PID
	float i_bl = PID_P / 12, i_br = PID_P / 12, i_fl = PID_P / 12, i_fr = PID_P / 12;

	//joystick declarations
	Joystick *driver, *op;

	//variables for translational auton movement
	float transfl = 0, transfr = 0, transbl = 0, transbr = 0, transavg = 0;

	//Encoder values when wheels are straight
	float LOWEST_ENC = 163;
	float HIGHEST_ENC = 3905;

	//Dorothy Comp
	float STRAIGHT_FL = 3340;//3467//1587
	float STRAIGHT_FR = 2859;//2863//988
	float STRAIGHT_BL = 1285;//1214//3160
	float STRAIGHT_BR = 400;//403//2283

	/*
	 //Hayley Prac
	 float STRAIGHT_FL = 1857; //1958
	 float STRAIGHT_FR = 543; //443
	 float STRAIGHT_BL = 457; //470
	 float STRAIGHT_BR = 2855; //2842
	 */
	float straight;

	//rotation angles for spinning
	float ANGLE_1 = 56.417;
	float ANGLE_2 = 123.583;
	float ANGLE_3 = 236.417;
	float ANGLE_4 = 303.583;

	//tote centric rotation
	float tote_ANGLE_1 = 118.989;
	float tote_ANGLE_2 = 61.011;
	float tote_ANGLE_3 = 78.272;
	float tote_ANGLE_4 = 101.728;

	//back centric rotation
	float back_ANGLE_1 = 78.272;
	float back_ANGLE_2 = 101.728;
	float back_ANGLE_3 = 118.989;
	float back_ANGLE_4 = 61.011;

	bool totecrot = 0;bool backcrot = 0;

	//Encoder values (previous)
	float flV, frV, blV, brV;

	//Encoder values (final after manipulation)
	float flFV, frFV, blFV, brFV;

	//boolean states and checks for encoder
	//states, 1="straight", 0="backwards"
	bool flPT, flTP, frPT, frTP, blPT, blTP, brPT, brTP;

	//Translationsl wheel speed
	float flSpeed = 0, frSpeed = 0, blSpeed = 0, brSpeed = 0;

	//forklift talons
	CANTalon *lift, *claw;

	//limit switches for forklift
	DigitalInput *dl, *dr, *tl, *tr;

	//forklift constants
	int fTar = 0, fEnc = 0, cTar = 0, cEnc = 0, fError = 0, cError = 0, count = 0;

	float fP1 = 205;//lowest height/first tote
	float fP2 = 1194;//2nd tote
	float fP3 = 2204;//3rd tote up
	float fP4 = 3170;//raise height from 2nd tote/4th tote
	float fP5 = 4150;//fifth tote
	float fP6 = 5552;//sixth tote

	float cP1 = 270;//narrow open
	float cP2 = 506;//Open
	float cP3 = 1;//closed

	//Forklift PID constants
	float fSpeed = 0, cSpeed = 0;
	float PID_P_F = 0.00275;
	float PID_P_C = 0.02;

	//claw I of PID
	int clawcumError = 0;
	float PID_I_C = .001;

	//Can grabber Solenoids for pistons
	Solenoid *armLA, *armLB, *armRA, *armRB;bool canit = 1;

	//Thunderclaps
	Solenoid *clapA, *clapB;

	//Direction for movement booleans
	bool flRev, frRev, blRev, brRev;

	//Translation PID constants
	float tError = 10;
	float tcumError = 0;
	float PID_I_T = 0;
	float tSpeed = 0;
	float aveaError = 0;

	//Gyro
	Gyro *gyro;
	float PID_gyro = .8;
	float gyroVal, gSpeed, gError, gTar;
	float gyrocumError = 0;
	float PID_I_G = 0;

	//current variables for wheels
	float flcurrent, frcurrent, blcurrent, brcurrent;

	//Boolean checks for states
	bool preset = false;//operator button 13
	bool clawPIDCheck = false;bool liftPIDCheck = false;bool bounce = false;//boolean to prevent bouncing on forklift-sets P for PID

	//Timer for second three tote
	Timer *time;

	//Timer for preset
	Timer *pTimer;

	//pdp used to detect current draw

	//multipurpose target for autons
	float auton_target = 0;
	float auton_target_back = 0;

	//Relay to send on/off command to LED lights
	Relay *ThunderGlow;

	void RobotInit()
	{
		chooser = new SendableChooser();
		//camera/vision initalization
		frame = imaqCreateImage(IMAQ_IMAGE_RGB, 0);
		imaqError = IMAQdxOpenCamera("cam1", IMAQdxCameraControlModeController, &session);
		camID = 0;
		//Joystick initialization
		driver = new Joystick(0);
		op = new Joystick(1);

		//compressor initialization
		comp = new Compressor(0);

		//Talon initialization for translational
		fl = new CANTalon(11);
		fr = new CANTalon(3);
		bl = new CANTalon(13);
		br = new CANTalon(2);

		//Talon initialization for rotational
		flS = new CANTalon(12);
		frS = new CANTalon(5);
		blS = new CANTalon(10);
		brS = new CANTalon(4);

		//Rotational encoders
		flSE = new AnalogInput(4);
		frSE = new AnalogInput(1);
		blSE = new AnalogInput(2);
		brSE = new AnalogInput(3);

		//Limit switches
		tl = new DigitalInput(6);
		tr = new DigitalInput(7);
		dl = new DigitalInput(8);
		dr = new DigitalInput(9);

		//Forklift and claw talon initialization
		lift = new CANTalon(0);
		claw = new CANTalon(9);

		//mast initialization
		mastA = new Solenoid(0);
		mastB = new Solenoid(1);

		//thunderclap initialization
		clapA = new Solenoid(2);
		clapB = new Solenoid(3);

		//Can grabber solenoid initialization
		armLA = new Solenoid(6);
		armLB = new Solenoid(7);
		armRA = new Solenoid(5);
		armRB = new Solenoid(4);
		//gyro initialization
		gyro = new AnalogGyro(new AnalogInput(0));

		//timer initialization
		time = new Timer();
		pTimer = new Timer();

		//PDP initialization for current draw
		pdp = new PowerDistributionPanel();

		//Relay initialization for on/off of LED lights
		ThunderGlow = new Relay(0);
	}

	void AutonomousInit()
	{
		//Reset auton values and classes
		ResetEverything();

		/*Start/Reset sensors*/
		time->Start();
		gyro->Reset();

		//Sets current translational encoders to 0
		fl->SetPosition(0);
		fr->SetPosition(0);
		bl->SetPosition(0);
		br->SetPosition(0);

		//auton initializations of state
		Twocan = grab2C;
		SecondThreeTote = prep1;
		OneCanAuton = closes;
		OneMoreTry = clamp1;
		autonCheck = true;
		//		onecan();
		twoCan();
		Twocan = drag2C;
		//		upandover();
	}
	void AutonomousPeriodic()
	{
		//TODO Select correct auton
//		onecan();
		twoCan();
//		upandover();
//		printf("time: %f", time->Get());
//		onemoretry();
//		printf("current: %f \n", pdp->GetCurrent(9));
	}

	void onemoretry()
	{
		if (autonCheck)
		{
			switch (OneMoreTry)
			{
				case clamp1:
					//closes thunderclaps
//				clapA->Set(0);
//				clapB->Set(1);
					claw->Set(-1);
					if (pdp->GetCurrent(9) > 6)
					{
						OneMoreTry = strafe1prep;
						claw->Set(0);
					}
					break;
				case strafe1prep:
					lift->Set(-1);
					RightEnc();
					aveaError = 0;
					aPID(flFV, flS, 90, 1, i_fl);
					aveaError += AV(aError / 4);
					aPID(frFV, frS, 90, 2, i_fr);
					aveaError += AV(aError / 4);
					aPID(blFV, blS, 90, 3, i_bl);
					aveaError += AV(aError / 4);
					aPID(brFV, brS, 90, 4, i_br);
					aveaError += AV(aError / 4);

					printf("lift: %i \n", lift->GetEncPosition());

					if (lift->GetEncPosition() > 1200)
					{
						lift->Set(0);
						fTar = lift->GetEncPosition();
						forkliftPID();
						OneMoreTry = strafe1;
						auton_target = 2160;
						auton_target_back = 1993;
						PID_auton = 1.0 / auton_target;
						gTar = 0;
					}
					break;
				case strafe1:
					forkliftPID();
					RightEnc();
					aveaError = 0;
					aPID(flFV, flS, 90, 1, i_fl);
					aveaError += AV(aError / 4);
					aPID(frFV, frS, 90, 2, i_fr);
					aveaError += AV(aError / 4);
					aPID(blFV, blS, 90, 3, i_bl);
					aveaError += AV(aError / 4);
					aPID(brFV, brS, 90, 4, i_br);
					aveaError += AV(aError / 4);

					transavg = 0;
					transfl = AV(fl->GetEncPosition());
					transavg += transfl / 4.0;
					transfr = AV(fr->GetEncPosition());
					transavg += transfr / 4.0;
					transbl = AV(bl->GetEncPosition());
					transavg += transbl / 4.0;
					transbr = AV(br->GetEncPosition());
					transavg += transbr / 4.0;

					printf("fl %f | fr %f | bl %f | br %f | angle %f \n", transfl, transfr, transbl, transbr, gyro->GetAngle());
//				gyroPID();
					gSpeed = .1;

					flSpeed = ((1 - gSpeed) * tlPID(auton_target, transfl, PID_auton));
					frSpeed = ((1 - gSpeed) * tlPID(auton_target, transfr, PID_auton));
					blSpeed = ((1 + gSpeed) * tlPID(auton_target_back, transbl, PID_auton));
					brSpeed = ((1 + gSpeed) * tlPID(auton_target_back, transbr, PID_auton));

					rev();

					fl->Set(-Deadband(flSpeed, .08));
					fr->Set(Deadband(frSpeed, .08));
					bl->Set(-Deadband(blSpeed, .08));
					br->Set(Deadband(brSpeed, .08));

					if (Deadband(auton_target - transavg, 100) == 0)
					{
						fl->Set(0);
						fr->Set(0);
						bl->Set(0);
						br->Set(0);
						fl->SetPosition(0);
						fr->SetPosition(0);
						bl->SetPosition(0);
						br->SetPosition(0);
						OneMoreTry = forward1prep;
					}
					break;
				case forward1prep:
					printf("forward1prep \n");
					forkliftPID();
					RightEnc();
					aveaError = 0;
					aPID(flFV, flS, .01, 1, i_fl);
					aveaError += AV(aError / 4);
					aPID(frFV, frS, .01, 2, i_fr);
					aveaError += AV(aError / 4);
					aPID(blFV, blS, .01, 3, i_bl);
					aveaError += AV(aError / 4);
					aPID(brFV, brS, .01, 4, i_br);
					aveaError += AV(aError / 4);

					if (aveaError < 2)
					{
						OneMoreTry = forward1;
						auton_target_back = 6480;
						auton_target = 6480 * 3.25 / 3.0;
						PID_auton = 1.0 / auton_target;
					}

					break;
				case forward1:
					printf("forward1 \n");
					forkliftPID();
					RightEnc();
					aveaError = 0;
					aPID(flFV, flS, .01, 1, i_fl);
					aveaError += AV(aError / 4);
					aPID(frFV, frS, .01, 2, i_fr);
					aveaError += AV(aError / 4);
					aPID(blFV, blS, .01, 3, i_bl);
					aveaError += AV(aError / 4);
					aPID(brFV, brS, .01, 4, i_br);
					aveaError += AV(aError / 4);

					transavg = 0;
					transfl = AV(fl->GetEncPosition());
					transavg += transfl / 4.0;
					transfr = AV(fr->GetEncPosition());
					transavg += transfr / 4.0;
					transbl = AV(bl->GetEncPosition());
					transavg += transbl / 4.0;
					transbr = AV(br->GetEncPosition());
					transavg += transbr / 4.0;

					gSpeed = .1;

					flSpeed = ((1 - gSpeed) * tlPID(auton_target, transfl, PID_auton));
					frSpeed = ((1 - gSpeed) * tlPID(auton_target, transfr, PID_auton));
					blSpeed = ((1 + gSpeed) * tlPID(auton_target_back, transbl, PID_auton));
					brSpeed = ((1 + gSpeed) * tlPID(auton_target_back, transbr, PID_auton));

					rev();

					fl->Set(-Deadband(flSpeed, .08));
					fr->Set(Deadband(frSpeed, .08));
					bl->Set(-Deadband(blSpeed, .08));
					br->Set(Deadband(brSpeed, .08));

					if (Deadband(auton_target - transavg, 100) == 0)
					{
						fl->Set(0);
						fr->Set(0);
						bl->Set(0);
						br->Set(0);
						fl->SetPosition(0);
						fr->SetPosition(0);
						bl->SetPosition(0);
						br->SetPosition(0);

						OneMoreTry = strafe2prep;
					}
					break;
				case strafe2prep:
					RightEnc();
					aveaError = 0;
					aPID(flFV, flS, 90, 1, i_fl);
					aveaError += AV(aError / 4);
					aPID(frFV, frS, 90, 2, i_fr);
					aveaError += AV(aError / 4);
					aPID(blFV, blS, 90, 3, i_bl);
					aveaError += AV(aError / 4);
					aPID(brFV, brS, 90, 4, i_br);
					aveaError += AV(aError / 4);

					if (aveaError < 2.0)
						printf("Done \n");
					break;

			}

		}
	}

	void upandover()
	{

		switch (SecondThreeTote)
		{

			case prep1:

				PID_I_CHECK = false;
				PID_gyro = .25;
				RightEnc();
				aveaError = 0;
				aPID(flFV, flS, 90, 1, i_fl);
				aveaError += AV(aError / 4);
				aPID(frFV, frS, 90, 2, i_fr);
				aveaError += AV(aError / 4);
				aPID(blFV, blS, 97, 3, i_bl);
				aveaError += AV(aError / 4);
				aPID(brFV, brS, 97, 4, i_br);
				aveaError += AV(aError / 4);

				cTar = 0;
				clawPID();

				if (Deadband(cSpeed, .1) == 0 && Deadband(aveaError, 8) == 0)
				{
					SecondThreeTote = up1;
					claw->Set(0);

				}

				break;

			case up1:

				mastA->Set(1);
				mastB->Set(0);

				auton_target = 6000;
				PID_auton = 1.0 / auton_target;

				fTar = fP4;
				forkliftPID();

				lift->Set(fSpeed);

				RightEnc();
				aveaError = 0;
				aPID(flFV, flS, 90, 1, i_fl);
				aveaError += AV(aError / 4);
				aPID(frFV, frS, 90, 2, i_fr);
				aveaError += AV(aError / 4);
				aPID(blFV, blS, 97, 3, i_bl);
				aveaError += AV(aError / 4);
				aPID(brFV, brS, 97, 4, i_br);
				aveaError += AV(aError / 4);

//			if (lift->GetEncPosition() > fP2){
//				transfl = AV(fl->GetEncPosition());
//				transfr = AV(fr->GetEncPosition());
//				transbl = AV(bl->GetEncPosition());
//				transbr = AV(br->GetEncPosition());
//
//				gyroPID();
//
//				flSpeed = .75*((1 + gSpeed) * tlPID(auton_target, transfl, PID_auton));
//				frSpeed = .75*((1 + gSpeed) * tlPID(auton_target, transfr, PID_auton));
//				blSpeed = .75*((1 - gSpeed) * tlPID(auton_target, transbl, PID_auton));
//				brSpeed = .75*((1 - gSpeed) * tlPID(auton_target, transbr, PID_auton));
//
//				rev();
//
//				fl->Set(Deadband(flSpeed, .08));
//				fr->Set(-Deadband(frSpeed, .08));
//				bl->Set(Deadband(blSpeed, .08));
//				br->Set(-Deadband(brSpeed, .08));
//
				if (Deadband(fSpeed, .1) == 0)
				{
					SecondThreeTote = over1;
					lift->Set(0);
				}

//			}

				break;

			case over1:

				if ((transfl + transfr + transbl + transbr) / 4.0 > 500)
					PID_auton = 1.0 / auton_target;
				else
					PID_auton = 1.0 / (1.5 * auton_target);
				printf("average %f fl %f fr %f bl %f br %f \n", (transfl + transfr + transbl + transbr) / 4.0, transfl, transfr, transbl, transbr);
				RightEnc();
				aveaError = 0;
				aPID(flFV, flS, 90, 1, i_fl);
				aveaError += AV(aError / 4);
				aPID(frFV, frS, 90, 2, i_fr);
				aveaError += AV(aError / 4);
				aPID(blFV, blS, 97, 3, i_bl);
				aveaError += AV(aError / 4);
				aPID(brFV, brS, 97, 4, i_br);
				aveaError += AV(aError / 4);

				printf("angle %f \n", gyro->GetAngle());

				transfl = AV(fl->GetEncPosition());
				transfr = AV(fr->GetEncPosition());
				transbl = AV(bl->GetEncPosition());
				transbr = AV(br->GetEncPosition());

				gyroPID();

				flSpeed = ((1 + gSpeed) * tlPID(auton_target, transfl, PID_auton));
				frSpeed = ((1 + gSpeed) * tlPID(auton_target, transfr, PID_auton));
				blSpeed = ((1 - gSpeed) * tlPID(auton_target, transbl, PID_auton));
				brSpeed = ((1 - gSpeed) * tlPID(auton_target, transbr, PID_auton));

				rev();

				fl->Set(Deadband(flSpeed, .08));
				fr->Set(-Deadband(frSpeed, .08));
				bl->Set(Deadband(blSpeed, .08));
				br->Set(-Deadband(brSpeed, .08));

				flcurrent = pdp->GetCurrent(12);
				frcurrent = pdp->GetCurrent(3);
				blcurrent = pdp->GetCurrent(13);
				brcurrent = pdp->GetCurrent(2);

				printf("flcurrent %f | frcurrent %f | blcurrent %f | brcurrent %f \n", flcurrent, frcurrent, blcurrent, brcurrent);

				if ((transfl + transfr + transbl + transbr) / 4.0 > 2500)
				{
					fTar = fP2 - 22;
					forkliftPID();
					lift->Set(fSpeed);

					if (Deadband(fSpeed, .15) == 0)
					{
						lift->Set(0);
					}
				}
				if ((transfl + transfr + transbl + transbr) / 4.0 > 5550)
				{
					fl->Set(0);
					fr->Set(0);
					bl->Set(0);
					br->Set(0);
//				SecondThreeTote = down1;
				}

				if (((Deadband(fSpeed, .2) == 0) && ((transfl + transfr + transbl + transbr) / 4.0 > 5550)) || time->Get() > 4.9)
				{
					fl->SetPosition(0);
					fr->SetPosition(0);
					bl->SetPosition(0);
					br->SetPosition(0);

					SecondThreeTote = down1;
				}

				break;
			case down1:

				claw->Set(1);
//			Wait(.8);
//			claw->Set(0);
				printf("clawcurrent %f | claw %i \n", pdp->GetCurrent(9), claw->GetEncPosition());
				if (pdp->GetCurrent(9) > 5)
				{
					claw->Set(0);
					fTar = 0;
					forkliftPID();
					lift->Set(fSpeed);

					if (Deadband(fSpeed, .05) == 0)
					{
						SecondThreeTote = clamp2;
						lift->Set(0);
					}

				}

				break;

			case clamp2:

				claw->Set(-1);

				if (pdp->GetCurrent(9) > 10)
				{
					claw->Set(0);
					SecondThreeTote = up2;
				}

				break;

			case up2:

//			mastA->Set(1);
//			mastB->Set(0);

				auton_target = 7380;
				PID_auton = 1.0 / auton_target;

				fTar = fP4;
				forkliftPID();

				lift->Set(fSpeed);

				RightEnc();
				aveaError = 0;
				aPID(flFV, flS, 90, 1, i_fl);
				aveaError += AV(aError / 4);
				aPID(frFV, frS, 90, 2, i_fr);
				aveaError += AV(aError / 4);
				aPID(blFV, blS, 97, 3, i_bl);
				aveaError += AV(aError / 4);
				aPID(brFV, brS, 97, 4, i_br);
				aveaError += AV(aError / 4);

//			if (lift->GetEncPosition() > (fP2 + fP1)/2){
//				transfl = AV(fl->GetEncPosition());
//				transfr = AV(fr->GetEncPosition());
//				transbl = AV(bl->GetEncPosition());
//				transbr = AV(br->GetEncPosition());
//
//				gyroPID();
//
//				flSpeed = ((1 + gSpeed) * tlPID(auton_target, transfl, PID_auton));
//				frSpeed = ((1 + gSpeed) * tlPID(auton_target, transfr, PID_auton));
//				blSpeed = ((1 - gSpeed) * tlPID(auton_target, transbl, PID_auton));
//				brSpeed = ((1 - gSpeed) * tlPID(auton_target, transbr, PID_auton));
//
//				rev();
//
//				fl->Set(Deadband(flSpeed, .08));
//				fr->Set(-Deadband(frSpeed, .08));
//				bl->Set(Deadband(blSpeed, .08));
//				br->Set(-Deadband(brSpeed, .08));
//
				if (Deadband(fSpeed, .12) == 0)
				{
					SecondThreeTote = over2;
					lift->Set(0);
				}
//
//			}

				break;

			case over2:
				printf("average %f fl %f fr %f bl %f br %f \n", (transfl + transfr + transbl + transbr) / 4.0, transfl, transfr, transbl, transbr);

				RightEnc();
				aveaError = 0;
				aPID(flFV, flS, 90, 1, i_fl);
				aveaError += AV(aError / 4);
				aPID(frFV, frS, 90, 2, i_fr);
				aveaError += AV(aError / 4);
				aPID(blFV, blS, 97, 3, i_bl);
				aveaError += AV(aError / 4);
				aPID(brFV, brS, 97, 4, i_br);
				aveaError += AV(aError / 4);

				transfl = AV(fl->GetEncPosition());
				transfr = AV(fr->GetEncPosition());
				transbl = AV(bl->GetEncPosition());
				transbr = (transfl + transfr + transbl) / 3.0;

				gyroPID();

				flSpeed = ((1 + gSpeed) * tlPID(auton_target, transfl, PID_auton));
				frSpeed = ((1 + gSpeed) * tlPID(auton_target, transfr, PID_auton));
				blSpeed = ((1 - gSpeed) * tlPID(auton_target, transbl, PID_auton));
				brSpeed = ((1 - gSpeed) * tlPID(auton_target, transbr, PID_auton));

				rev();

				fl->Set(Deadband(flSpeed, .08));
				fr->Set(-Deadband(frSpeed, .08));
				bl->Set(Deadband(blSpeed, .08));
				br->Set(-Deadband(brSpeed, .08));

				printf("fSpeed: %f \n", fSpeed);
				if ((transfl + transfr + transbl + transbr) / 4.0 > 4000)
				{
					fTar = fP2 - 30;
					forkliftPID();
					lift->Set(fSpeed);
//				mastA->Set(0);
//				mastB->Set(1);

					if (Deadband(fSpeed, .05) == 0)
					{
						lift->Set(0);
					}
				}
				if ((transfl + transfr + transbl + transbr) / 4.0 > 6530)
				{
					fl->Set(0);
					fr->Set(0);
					bl->Set(0);
					br->Set(0);
				}

				if ((Deadband(fSpeed, .15) == 0) && ((transfl + transfr + transbl + transbr) / 4.0 > 6530))
				{
					fl->SetPosition(0);
					fr->SetPosition(0);
					bl->SetPosition(0);
					br->SetPosition(0);

					SecondThreeTote = down2;
				}

				break;

			case down2:
//			cTar = -200;
//			clawPID();
				claw->Set(1);
				mastA->Set(0);
				mastB->Set(1);

				if (pdp->GetCurrent(9) > 8)
				{
					claw->Set(0);
					fTar = 0;
					forkliftPID();
					lift->Set(fSpeed);

					if (Deadband(fSpeed, .15) == 0)
					{
						SecondThreeTote = clamp3;
						lift->Set(0);
						printf("time %f", time->Get());
						clawcumError = 0;
					}

				}

				break;

			case clamp3:
//
				cTar = 50;
				clawPID();
//			claw->Set(-1);
//			printf("clawcurrent %f", pdp->GetCurrent(9));
				if (Deadband(cSpeed, .1) == 0)
				{
					claw->Set(0);
					SecondThreeTote = up3;
					fl->SetPosition(0);
					fr->SetPosition(0);
					bl->SetPosition(0);
					br->SetPosition(0);
				}

				break;

			case up3:

				auton_target = 10000;
				PID_auton = 2.0 / auton_target;

				lift->Set(-1);

				RightEnc();
				aveaError = 0;
				aPID(flFV, flS, 1, 1, i_fl);
				aveaError += AV(aError / 4);
				aPID(frFV, frS, 1, 2, i_fr);
				aveaError += AV(aError / 4);
				aPID(blFV, blS, 1, 3, i_bl);
				aveaError += AV(aError / 4);
				aPID(brFV, brS, 1, 4, i_br);
				aveaError += AV(aError / 4);

//			if (lift->GetEncPosition() > (fP2 + fP1)/2){
				transfl = AV(fl->GetEncPosition());
				transfr = AV(fr->GetEncPosition());
				transbl = AV(bl->GetEncPosition());
				transbr = (transfl + transfr + transbl) / 3.0;

				gyroPID();

				flSpeed = ((1 - gSpeed) * tlPID(auton_target, transfl, PID_auton));
				frSpeed = ((1 + gSpeed) * tlPID(auton_target, transfr, PID_auton));
				blSpeed = ((1 - gSpeed) * tlPID(auton_target, transbl, PID_auton));
				brSpeed = ((1 + gSpeed) * tlPID(auton_target, transbr, PID_auton));

				rev();

				fl->Set(-Deadband(flSpeed, .08));
				fr->Set(Deadband(frSpeed, .08));
				bl->Set(-Deadband(blSpeed, .08));
				br->Set(Deadband(brSpeed, .08));

				if (lift->GetEncPosition() > 300)
				{
					SecondThreeTote = back;
					lift->Set(0);
				}
//
//			}

				break;

			case back:

				RightEnc();
				aveaError = 0;
				aPID(flFV, flS, 1, 1, i_fl);
				aveaError += AV(aError / 4);
				aPID(frFV, frS, 1, 2, i_fr);
				aveaError += AV(aError / 4);
				aPID(blFV, blS, 1, 3, i_bl);
				aveaError += AV(aError / 4);
				aPID(brFV, brS, 1, 4, i_br);
				aveaError += AV(aError / 4);

				transfl = AV(fl->GetEncPosition());
				transfr = AV(fr->GetEncPosition());
				transbl = AV(bl->GetEncPosition());
				transbr = (transfl + transfr + transbl) / 3.0;

				gyroPID();

				flSpeed = ((1 - gSpeed) * tlPID(auton_target, transfl, PID_auton));
				frSpeed = ((1 + gSpeed) * tlPID(auton_target, transfr, PID_auton));
				blSpeed = ((1 - gSpeed) * tlPID(auton_target, transbl, PID_auton));
				brSpeed = ((1 + gSpeed) * tlPID(auton_target, transbr, PID_auton));

				rev();

				fl->Set(-Deadband(flSpeed, .08));
				fr->Set(Deadband(frSpeed, .08));
				bl->Set(-Deadband(blSpeed, .08));
				br->Set(Deadband(brSpeed, .08));

				if ((transfl + transfr + transbl + transbr) / 4.0 > 9000)
				{
					fl->Set(0);
					fr->Set(0);
					bl->Set(0);
					br->Set(0);
					SecondThreeTote = open2;
				}

				break;

			case open2:

				fTar = 0;
				forkliftPID();
				lift->Set(fSpeed);

//			if(Deadband(fSpeed, .05) ==0){
//				lift->Set(0);
//				cTar = -137;
//				clawPID();

				if (dl == 0 || dr == 0)
				{
					lift->Set(0);
				}

				claw->Set(1);
				if (pdp->GetCurrent(9) > 8)
				{
					claw->Set(0);
					SecondThreeTote = finish2;
				}

				break;

			case finish2:
				auton_target = 100;
				PID_auton = 1.0 / auton_target;

				RightEnc();
				aveaError = 0;
				aPID(flFV, flS, 1, 1, i_fl);
				aveaError += AV(aError / 4);
				aPID(frFV, frS, 1, 2, i_fr);
				aveaError += AV(aError / 4);
				aPID(blFV, blS, 1, 3, i_bl);
				aveaError += AV(aError / 4);
				aPID(brFV, brS, 1, 4, i_br);
				aveaError += AV(aError / 4);

				transfl = AV(fl->GetEncPosition());
				transfr = AV(fr->GetEncPosition());
				transbl = AV(bl->GetEncPosition());
				transbr = (transfl + transfr + transbl) / 3.0;

				gyroPID();

				flSpeed = ((1 - gSpeed) * tlPID(auton_target, transfl, PID_auton));
				frSpeed = ((1 + gSpeed) * tlPID(auton_target, transfr, PID_auton));
				blSpeed = ((1 - gSpeed) * tlPID(auton_target, transbl, PID_auton));
				brSpeed = ((1 + gSpeed) * tlPID(auton_target, transbr, PID_auton));

				rev();

				fl->Set(-Deadband(flSpeed, .05));
				fr->Set(Deadband(frSpeed, .05));
				bl->Set(-Deadband(blSpeed, .05));
				br->Set(Deadband(brSpeed, .05));

				if ((transfl + transfr + transbl + transbr) / 4.0 > 45)
				{

					fl->Set(0);
					fr->Set(0);
					bl->Set(0);
					br->Set(0);
				}

				break;
		}

	}

	void twoCan()//auton that grabs two cans from center and drags them back
	{

		if (autonCheck == true)//ensures auton only goes once
		{
			switch (Twocan)
			{
				case grab2C://drops can grabbers
					PID_I_CHECK = true;//enables use of I from PID

					//drops left arm
					armLA->Set(1);
					armLB->Set(0);

					//drops right arm
					armRA->Set(0);
					armRB->Set(1);

					Wait(1);//.3 on ramp,.15 on flat

					Twocan = drag2C;
					break;

				case drag2C://drive forwards dragging cans with us
					//constants set for translationional movement
					auton_target = 6900;//short drag=6900/2-500
					gTar = 0;
					PID_gyro = 0.5;
					PID_I_G = 0;
					PID_auton = 1.0 / auton_target;

					//Wheels rotated to frontwards orientation
					RightEnc();
					aveaError = 0;
					aPID(flFV, flS, 1, 1, i_fl);
					aveaError += AV(aError / 4);
					aPID(frFV, frS, 1, 2, i_fr);
					aveaError += AV(aError / 4);
					aPID(blFV, blS, 1, 3, i_bl);
					aveaError += AV(aError / 4);
					aPID(brFV, brS, 1, 4, i_br);
					aveaError += AV(aError / 4);

					//translational variables given current encoder value
					transfl = AV(fl->GetEncPosition());
					transfr = AV(fr->GetEncPosition());
					transbl = AV(bl->GetEncPosition());
					transbr = AV(br->GetEncPosition());

					//speed of translational talons set
					flSpeed = 1.3 * (tlPID(auton_target, transfl, PID_auton));
					frSpeed = 1.3 * (tlPID(auton_target, transfr, PID_auton));
					blSpeed = 1.3 * (tlPID(auton_target, transbl, PID_auton));
					brSpeed = 1.3 * (tlPID(auton_target, transbr, PID_auton));
					rev();

					//Condition to set talons to 0 when it goes under a certain speed or reaches a certain distance
					fl->Set(Deadband(flSpeed, .08));
					fr->Set(-Deadband(frSpeed, .08));
					bl->Set(Deadband(blSpeed, .08));
					br->Set(-Deadband(brSpeed, .08));
					if ((transfl + transfr + transbl + transbr) / 4.0 > auton_target)
					{
						fl->Set(0);
						fr->Set(0);
						bl->Set(0);
						br->Set(0);
						lift->SetPosition(0);
						Twocan = up2C;
					}
					break;

				case up2C://raises can grabbers

					//left arm goes up
					armLA->Set(0);
					armLB->Set(1);

					//right arm goes up
					armRA->Set(1);
					armRB->Set(0);
					Twocan = forksdown;

					break;

				case forksdown:
					//lowers forks close to bottom
					fTar = -5400;
					forkliftPID();
					lift->Set(fSpeed);
					//puts mast forward
					mastA->Set(0);
					mastB->Set(1);
					if (fEnc < -5100)
					{
						//ensures auton only runs once
						autonCheck = false;
						lift->Set(0);
					}
					break;
			}
		}
	}

	void gyroPID()//PID used to keep us moving in a straight line
	{
		//obtains current gyrovalue and calculates how much off we are from the target
		gyroVal = gyro->GetAngle();
		gError = gTar - gyroVal;

		//Utilises error to increase the value of cumError for use of I in PID
		if (AV(gError) < 10)
		{
			gyrocumError += gError;
		}

		//Sets value for gSpeed which is used in translational movement
		gSpeed = gError * PID_gyro + gyrocumError * PID_I_G;
		printf("%f gyroVal %f gTar %f gyroCumError, %f PID_I_G %f gSpeed \n", gyroVal, gTar, gyrocumError, PID_I_G, gSpeed);
	}

	void onecan()//auton to grab one can from setup zone,drag back, and drop off
	{
		if (autonCheck)//ensures auton only runs once
		{
			switch (OneCanAuton)
			{
				case closes://closes on garbage can

					//Sets PID constants for claw
					PID_P_C = 0.10;
					cTar = 340;

					//close claw
					clawPID();
					claw->Set(cSpeed);
					if (Deadband(cSpeed, 0.45) == 0 || pdp->GetCurrent(9) >= 6 || time->Get() > 3)
					{
						claw->Set(0);
						EncReset();
						OneCanAuton = raise2;
					}

					break;

				case raise2://raises garbage can
					fTar = 720;
					forkliftPID();
					lift->Set(fSpeed);
					if (Deadband(fSpeed, 0.25) == 0 || time->Get() > 7)
					{
						lift->Set(0);
						OneCanAuton = back1;
						gyro->Reset();
						EncReset();
						tcumError = 0;
					}
					break;

				case back1://going backwards to auto zone
					//PID constants
					auton_target = 7450;
					gTar = 0;
					PID_gyro = 0.5;
					PID_I_G = 0;
					PID_auton = 1.0 / auton_target;

					//orientate wheels to straight
					RightEnc();
					aveaError = 0;
					aPID(flFV, flS, 1, 1, i_fl);
					aveaError += AV(aError / 4);
					aPID(frFV, frS, 1, 2, i_fr);
					aveaError += AV(aError / 4);
					aPID(blFV, blS, 1, 3, i_bl);
					aveaError += AV(aError / 4);
					aPID(brFV, brS, 1, 4, i_br);
					aveaError += AV(aError / 4);

					//set variables to current translational encoder values
					transfl = AV(fl->GetEncPosition());
					transfr = AV(fr->GetEncPosition());
					transbl = AV(bl->GetEncPosition());
					transbr = AV(br->GetEncPosition());

					//Give talons throttle based on translational pid and gyropid
					flSpeed = ((1 + gSpeed) * tlPID(auton_target, transfl, PID_auton));
					frSpeed = ((1 - gSpeed) * tlPID(auton_target, transfr, PID_auton));
					blSpeed = ((1 + gSpeed) * tlPID(auton_target, transbl, PID_auton));
					brSpeed = ((1 - gSpeed) * tlPID(auton_target, transbr, PID_auton));
					rev();//determines direction throttle is applied

					//sets translational talons to 0 if speed gets low enough
					fl->Set(-Deadband(flSpeed, .08));
					fr->Set(Deadband(frSpeed, .08));
					bl->Set(-Deadband(blSpeed, .08));
					br->Set(Deadband(brSpeed, .08));

					//stops translational and rotational talons if either distance or time condition is met
					if ((transfl + transfr + transbl) / 3.0 > auton_target || time->Get() > 11)
					{
						fl->Set(0);
						fr->Set(0);
						bl->Set(0);
						br->Set(0);
						flS->Set(0);
						frS->Set(0);
						blS->Set(0);
						brS->Set(0);
						gyrocumError = 0;
						OneCanAuton = turn45prep;
					}

					break;

				case turn45prep://turn wheels to go the right direction (turn left/right)
					//orientate wheels to correct orientation for rotating
					RightEnc();
					aveaError = 0;
					aPID(flFV, flS, ANGLE_1, 1, i_fl);
					aveaError += AV(aError / 4);
					aPID(frFV, frS, ANGLE_2, 2, i_fr);
					aveaError += AV(aError / 4);
					aPID(blFV, blS, ANGLE_4, 3, i_bl);
					aveaError += AV(aError / 4);
					aPID(brFV, brS, ANGLE_3, 4, i_br);
					aveaError += AV(aError / 4);

					//stops rotational talons if condition average pid error is less than certain amount
					if (Deadband(aveaError, 8) == 0)
					{
						clawcumError = 0;
						OneCanAuton = turn45;
						flS->Set(0);
						frS->Set(0);
						blS->Set(0);
						brS->Set(0);
						gyro->Reset();
						EncReset();
						tcumError = 0;
					}
					break;

				case turn45://turn left/right
					//Set gyro pid constants
					PID_gyro = 1.0 / 30.0;
					PID_I_G = PID_gyro * .1;

					//rotates rotation talons to correct orientation for spinning
					RightEnc();
					aPID(flFV, flS, ANGLE_1, 1, i_fl);
					aPID(frFV, frS, ANGLE_2, 2, i_fr);
					aPID(blFV, blS, ANGLE_4, 3, i_bl);
					aPID(brFV, brS, ANGLE_3, 4, i_br);

					//set gyro pid constants
					gTar = 45;
					gyroPID();

					//throttle for wheels given from gyropid, condition based on orientation of wheels
					if (flRev)
					{
						fl->Set(-1 * gSpeed);
					}
					else
					{
						fl->Set(gSpeed);
					}
					if (frRev)
					{
						fr->Set(gSpeed);
					}
					else
					{
						fr->Set(-1 * gSpeed);
					}
					if (blRev)
					{
						bl->Set(-1 * gSpeed);
					}
					else
					{
						bl->Set(gSpeed);
					}
					if (brRev)
					{
						br->Set(gSpeed);
					}
					else
					{
						br->Set(-1 * gSpeed);
					}

					//condition gyro is within certain range and speed is low enough
					if (Deadband(AV(gyroVal - 45), 1) == 0 && Deadband(gSpeed, .1) == 0)
					{
						//stop translational and rotational talons
						gyrocumError = 0;
						fl->Set(0);
						fr->Set(0);
						bl->Set(0);
						br->Set(0);

						flS->Set(0);
						frS->Set(0);
						blS->Set(0);
						brS->Set(0);
						lift->Set(0);
						OneCanAuton = endOneCan;
					}

					break;

				case endOneCan://sets down can
					//set forklift pid constants
					fTar = 50;
					forkliftPID();
					lift->Set(fSpeed);
					if (fEnc <= 55)
					{
						lift->Set(0);
						//open claws
						cTar = 5;
						clawPID();
						if (cEnc < 10)
						{
							claw->Set(0);
							OneCanAuton = teleopprep;
						}
					}
					break;

				case teleopprep:

					fTar = -700;
					forkliftPID();
					lift->Set(fSpeed);
					if (fEnc < -680)
					{
						lift->Set(0);
						autonCheck = false;

					}
					break;
			}
		}
	}
	//translational PID, will be used for auton
	float tlPID(float encTar, float encPos, float PID)
	{
		float tlpidfin = 0;

		//determine whether to use i or not used between autons
		if (PID_I_CHECK)
		{
			PID_I_T = PID * .01;
		}
		else
		{
			PID_I_T = 0;
		}
		//translational pid
		tError = encTar - encPos;
		tcumError += tError / 4;
		tlpidfin = tError * PID + tcumError * PID_I_T;
		//limit output to talons to its max
		if (tlpidfin > 1)
		{
			tlpidfin = 1;
		}
		return (tlpidfin);
	}
	void TeleopInit()
	{
		//Turn on led lights
		//ThunderGlow->Set(Relay::kForward);

		//Bring left can grabber up
//		armLA->Set(0);
//		armLB->Set(1);

		//Bring right can grabber up
		//armRA->Set(1);
		//armRB->Set(0);

		//resets most sensor values and conditions
		ResetEverything();

		//compressor is off
		compOn = false;

		//camera code
		imaqError = IMAQdxConfigureGrab(session);

		//allow it to fill if needed
		comp->SetClosedLoopControl(1);
	}

	void TeleopPeriodic()
	{
		mastA->Get();
		//	printf("%i mastA %i mastB %i clapA %i clapB %i armRA %i armRB %i armLA %i armLB", mastA->Get(), mastB->Get(), clapA->Get(), clapB->Get(), armRA->Get(), armRB->Get(), armLA->Get(), armLB->Get());
//		printf("%f fl %f fr %f bl %f br %i claw %i lift %f current \n", fl->GetPosition(), fr->GetPosition(), bl->GetPosition(), br->GetPosition(), claw->GetEncPosition(), lift->GetEncPosition(), pdp->GetCurrent(9));
		printf("%f fls %f frs %f bls %f brs \n", flFV, frFV, blFV, brFV);
		printf("fl: %i | fr: %i | bl: %i | br: %i \n", flSE->GetValue(), frSE->GetValue(), blSE->GetValue(), brSE->GetValue());
		//vision streaming code
		IMAQdxStartAcquisition(session);
		IMAQdxGrab(session, frame, true, NULL);
		if (imaqError != IMAQdxErrorSuccess)
		{
			DriverStation::ReportError("IMAQdxGrab error: " + std::to_string((long) imaqError) + "\n");
		}
		else
		{
			imaqDrawShapeOnImage(frame, frame, { 75, 90, 195, 280 }, DrawMode::IMAQ_DRAW_VALUE, ShapeMode::IMAQ_SHAPE_RECT, 5);//0.0f
			CameraServer::GetInstance()->SetImage(frame);
			Wait(0.005);
		}

		//master mast control
		Mast();

		//give encoders correct values
		RightEnc();

		//master teleop driver control
		Drivebase();

		//master teleop operator control
		Forklift();

		//ThunderClap master and can grabber manual
		ThunderClap();

	}
	void ThunderClap()
	{
		if (fEnc < 1200)
		{
			clapA->Set(1);
			clapB->Set(0);
		}
		//open thunderclaps
		if (fEnc >= 1200)
		{
			if (driver->GetRawButton(8))
			{
				clapA->Set(1);
				clapB->Set(0);
			}
			//close thunderclaps
			if (driver->GetRawButton(6))
			{
				clapA->Set(0);
				clapB->Set(1);
			}
		}
		//open forklift and thunderclaps
		if (op->GetRawButton(14))
		{
			clapA->Set(1);
			clapB->Set(0);
			cTar = 40;
			clawPIDCheck = true;
			clawPID();
		}

		//drop can grabber
		if (driver->GetRawButton(14))
		{
			//drop left arm
			armLA->Set(1);
			armLB->Set(0);

			//drop right arm
			armRA->Set(0);
			armRB->Set(1);
		}
		//pick up can grabber
		if (driver->GetRawButton(13))
		{
			//raises left arm
			armLA->Set(0);
			armLB->Set(1);

			//raises right arm
			armRA->Set(1);
			armRB->Set(0);
		}

	}
	void Mast()//control mast/pneumatics on bot
	{
		//turn compressor on
		if (driver->GetRawButton(14) && !compOn)
		{
			compOn = true;
			comp->SetClosedLoopControl(1);
		}
		//turn compressor off
		else
			if (driver->GetRawButton(14) && compOn)
			{
				compOn = false;
				comp->SetClosedLoopControl(0);
			}
		//toggle for mast
		if (op->GetRawButton(5) || op->GetRawButton(7))
		{
			//Left bumper, sets mast forward
			if (op->GetRawButton(5))
			{
				mastA->Set(1);
				mastB->Set(0);
			}
			//left trigger, sets mast backward
			else
				if (op->GetRawButton(7))
				{
					mastA->Set(0);
					mastB->Set(1);
				}
		}
	}
	void Forklift()//operator control of forklift
	{
		fEnc = lift->GetEncPosition();
		cEnc = -1 * claw->GetEncPosition();

		//Right trigger, narrow tote grab
		if (op->GetRawButton(8))
		{
			cTar = cP2;
			clawPIDCheck = true;

		}
		//options button, resets claw encoder
		if (op->GetRawButton(10))
		{
			claw->SetPosition(0);
			cTar = cEnc;
		}

		//right bumper, extends claw out
		if (op->GetRawButton(6))
		{
			cTar = 190;
			clawPIDCheck = true;
		}

		//claw code
		//Left and rightf on right stick of operator controller
		if (Deadband(op->GetZ(), 0.15) != 0)
		{
			cSpeed = op->GetZ();
			cTar = cEnc;
			claw->Set(cSpeed);
			clawPIDCheck = false;
		}
		else
			if (clawPIDCheck)
			{
				clawPID();
			}
			else
			{
				claw->Set(0);
			}

		//forklift code
		//Up and down on left stick of operator controller
		//Square
		if (op->GetRawButton(1))
		{
			fTar = fP1;
			liftPIDCheck = true;
		}
		if (op->GetRawButton(2))
		{
			fTar = fP2;
			liftPIDCheck = true;
		}
		//Circle
		if (op->GetRawButton(3))
		{
			fTar = fP3;
			liftPIDCheck = true;
		}
		//Triangle
		if (op->GetRawButton(4))
		{
			fTar = fP4;
			liftPIDCheck = true;
		}
		//Share button
		if (op->GetRawButton(9))
		{
			fTar = fP5;
			liftPIDCheck = true;
		}

		if (op->GetRawButton(13))
		{
			preset = true;
			pTimer->Reset();
			pTimer->Start();
			drop = p3;
		}
		else
			if (preset == false)
			{
				//manual control of forklift
				if (Deadband(op->GetY(), 0.05) != 0)
				{
					preset = false;
					fSpeed = op->GetY();
					fTar = lift->GetEncPosition();
					forkliftLimit();
					lift->Set(Deadband(fSpeed, 0.05));
					bounce = false;
					PID_P_F = 0;
					count = 0;
				}
				else
				{
					count++;
					if (!bounce && count > 10)
					{
						PID_P_F = 0.00275;
						fTar = lift->GetEncPosition();
						bounce = true;

					}
					forkliftPID();
					forkliftLimit();
					lift->Set(Deadband(fSpeed, 0.05));

				}
			}
		//operator auto stack on human player station
		if (preset)
		{
			switch (drop)
			{
				//set forklift to starting position on third tote
				case p3:
					mastA->Set(0);
					mastB->Set(1);
					fTar = fP3;//fP2
					forkliftPID();
					lift->Set(fSpeed * 0.85);
					if (Deadband(fSpeed, 0.25) == 0 || pTimer->Get() >= 2)
					{
						pTimer->Reset();
						printf("%f PRESET TIMER P3 STATE\n", pTimer->Get());
						lift->Set(0);
						drop = open;

					}
					//manual cancel auto stack
					if ((op->GetRawButton(12)) || (Deadband(op->GetZ(), 0.15) != 0) || (Deadband(op->GetY(), 0.05) != 0))
					{
						preset = false;
						clawPIDCheck = false;
					}
					break;
					//open claw
				case open:
					cTar = 250;//390
					PID_I_C = 0;
					clawPID();
					if (cEnc < 270 || pTimer->Get() >= 1.1)//370
					{
						pTimer->Reset();
						claw->Set(0);
						drop = p2;
						printf("%f PRESET TIMER OPEN STATE\n", pTimer->Get());
					}
					//manual cancel auto stack
					if ((op->GetRawButton(12)) || (Deadband(op->GetZ(), 0.15) != 0) || (Deadband(op->GetY(), 0.05) != 0))
					{
						preset = false;
						clawPIDCheck = false;
					}
					break;
					//bring forklift down to grab bottom tote
				case p2:
					fTar = fP1;//1200
					forkliftPID();
					lift->Set(0.7 * fSpeed);
					if (fEnc <= 350 || pTimer->Get() >= 2)
					{
						pTimer->Reset();
						lift->Set(0);
						drop = close_preset;
						printf("%f PRESET TIMER P2 STATE\n", pTimer->Get());
					}
					//manual cancel auto stack
					if ((op->GetRawButton(12)) || (Deadband(op->GetZ(), 0.15) != 0) || (Deadband(op->GetY(), 0.05) != 0))
					{
						preset = false;
						clawPIDCheck = false;
					}
					break;
					//close claw
				case close_preset:
					cTar = 720;
					clawPID();
					if (cEnc >= 495 || pTimer->Get() >= 2)
					{
						pTimer->Reset();
						claw->Set(0);
						drop = p4;
						printf("%f PRESET TIMER CLOSE STATE\n", pTimer->Get());
					}
					//manual cancel auto stack
					if ((op->GetRawButton(12)) || (Deadband(op->GetZ(), 0.15) != 0) || (Deadband(op->GetY(), 0.05) != 0))
					{
						preset = false;
						clawPIDCheck = false;
					}
					break;
					//raise forklift from 2nd tote to 4th tote height
				case p4:
					mastA->Set(1);
					mastB->Set(0);
					fTar = fP4;
					forkliftPID();
					lift->Set(fSpeed * 0.85);
					if (Deadband(fSpeed, 0.25) == 0)
					{
						lift->Set(0);
						printf("%f PRESET TIMER P4 STATE\n", pTimer->Get());
						preset = false;
						clawPIDCheck = false;
					}
					//manual cancel auto stack
					if ((op->GetRawButton(12)) || (Deadband(op->GetZ(), 0.15) != 0) || (Deadband(op->GetY(), 0.05) != 0))
					{
						preset = false;
						clawPIDCheck = false;
					}
					break;
			}
		}
	}

	//makes the wheels 168-3000
	void RightEnc()
	{
		//initial values set
		flV = HIGHEST_ENC - (float) flSE->GetValue() + LOWEST_ENC;
		frV = HIGHEST_ENC - (float) frSE->GetValue() + LOWEST_ENC;
		blV = HIGHEST_ENC - (float) blSE->GetValue() + LOWEST_ENC;
		brV = HIGHEST_ENC - (float) brSE->GetValue() + LOWEST_ENC;

		//final values set straighten-changes value range, encMath-turns into degrees
		flFV = straighten(flV, STRAIGHT_FL);
		flFV = encMath(flFV);
		frFV = straighten(frV, STRAIGHT_FR);
		frFV = encMath(frFV);
		blFV = straighten(blV, STRAIGHT_BL);
		blFV = encMath(blFV);
		brFV = straighten(brV, STRAIGHT_BR);
		brFV = encMath(brFV);

	}

	//absolute value for floats
	float AV(float num)
	{
		if (num <= 0)
		{
			return num * -1;
		}
		return num;
	}
	//swerve drive code
	void Drivebase()
	{
		if (driver->GetRawButton(7))
			totecrot = 0;
		else
			totecrot = 1;

		if (driver->GetRawButton(5))
		{
			backcrot = 0;
		}
		else
			backcrot = 1;
		//obtain raw input
		mag = driver->GetMagnitude();
		angleD = driver->GetDirectionDegrees();
		//Makes angle +
		if (angleD < 0)
		{
			angleD = angleD + 360.0;
		}
		//set talon speed for translational
		if (Deadband(mag, 0.08) != 0)
		{
			mix = false;
			flSpeed = mag;
			frSpeed = mag;
			blSpeed = mag;
			brSpeed = mag;

			//circle, aligns for side-to-side movement
			if (driver->GetRawButton(3))
			{
				//applied throttle for strafing
				if (driver->GetX() > 0)
				{
					//direction of throttle values set for rev
					flRev = false;
					frRev = false;
					blRev = false;
					brRev = false;

					//rotate wheels to strafe position
					aPID(flFV, flS, 90, 1, i_fl);
					aPID(frFV, frS, 90, 2, i_fr);
					aPID(blFV, blS, 90, 3, i_bl);
					aPID(brFV, brS, 90, 4, i_br);
				}
				else
				{
					//direction of throttle values set for rev
					flRev = true;
					frRev = true;
					blRev = true;
					brRev = true;

					//rotate wheels to strafe position
					aPID(flFV, flS, 270, 1, i_fl);
					aPID(frFV, frS, 270, 2, i_fr);
					aPID(blFV, blS, 270, 3, i_bl);
					aPID(brFV, brS, 270, 4, i_br);
				}

				rev();
			}
			else
			{
				//rotate to manual given value
				aPID(flFV, flS, angleD, 1, i_fl);
				aPID(frFV, frS, angleD, 2, i_fr);
				aPID(blFV, blS, angleD, 3, i_bl);
				aPID(brFV, brS, angleD, 4, i_br);
				rev();
			}
			//set translational talon speed "capped at 85%
			fl->Set(0.85 * flSpeed);
			fr->Set(0.85 * -frSpeed);
			bl->Set(0.85 * blSpeed);
			br->Set(0.85 * -brSpeed);

		}
		else
		{
			//circle, aligns for side-to-side movement
			if (driver->GetRawButton(3))
			{
				//direction of throttle values set for rev

				flRev = false;
				frRev = false;
				blRev = false;
				brRev = false;

				//rotate wheels to strafe position
				aPID(flFV, flS, 90, 1, i_fl);
				aPID(frFV, frS, 90, 2, i_fr);
				aPID(blFV, blS, 90, 3, i_bl);
				aPID(brFV, brS, 90, 4, i_br);
				rev();

			}
			//other strafe button
			else
				if (driver->GetRawButton(4))
				{
					//direction of throttle values set for rev
					flRev = true;
					frRev = true;
					blRev = true;
					brRev = true;

					//rotate wheels to strafe position
					aPID(flFV, flS, 270, 1, i_fl);
					aPID(frFV, frS, 270, 2, i_fr);
					aPID(blFV, blS, 270, 3, i_bl);
					aPID(brFV, brS, 270, 4, i_br);

					rev();
				}
				else
				{
					//straighten wheels
					aPID(flFV, flS, flFV, 1, i_fl);
					aPID(frFV, frS, frFV, 2, i_fr);
					aPID(blFV, blS, blFV, 3, i_bl);
					aPID(brFV, brS, brFV, 4, i_br);
					rev();
				}

			//set translational talon throttle to 0
			fl->Set(0);
			fr->Set(0);
			bl->Set(0);
			br->Set(0);

		}

		//X, aligns for forward-backward movement
		if (driver->GetRawButton(2))
		{
			//straighten wheels
			aPID(flFV, flS, 1, 1, i_fl);
			aPID(frFV, frS, 1, 2, i_fr);
			aPID(blFV, blS, 1, 3, i_bl);
			aPID(brFV, brS, 1, 4, i_br);

		}
		//reset encoders manually
		if (driver->GetRawButton(14))
		{
			EncReset();
		}
		//reset gyro manually
		if (driver->GetRawButton(7))
		{
			gyro->Reset();
		}

		//Snake Calculations
//		if(mag != 0 && driver->GetZ() != 0){
//			snakeOn = true;
//			Snake();
//				}
//		else
//		{
		//left and right on right stick of driver controller:rotation
		if (driver->GetZ() >= 0.08)//&& Deadband(driver->GetY(),.05)==0
		{
			mix = true;
			//rotate wheels to rotationary position

			if (!totecrot)
			{
				aPID(flFV, flS, tote_ANGLE_1, 1, i_fl);
				aPID(frFV, frS, tote_ANGLE_2, 2, i_fr);
				aPID(blFV, blS, tote_ANGLE_4, 3, i_bl);
				aPID(brFV, brS, tote_ANGLE_3, 4, i_br);
			}
			else
				if (!backcrot)
				{
					aPID(flFV, flS, back_ANGLE_1, 1, i_fl);
					aPID(frFV, frS, back_ANGLE_2, 2, i_fr);
					aPID(blFV, blS, back_ANGLE_4, 3, i_bl);
					aPID(brFV, brS, back_ANGLE_3, 4, i_br);
				}
				else
				{
					aPID(flFV, flS, ANGLE_1, 1, i_fl);
					aPID(frFV, frS, ANGLE_2, 2, i_fr);
					aPID(blFV, blS, ANGLE_4, 3, i_bl);
					aPID(brFV, brS, ANGLE_3, 4, i_br);
				}
			//set translational talon throttle
			fl->Set(0.85 * Deadband(driver->GetZ(), 0.08));
			fr->Set(0.85 * -Deadband(driver->GetZ(), 0.08));
			bl->Set(0.85 * Deadband(driver->GetZ(), 0.08));
			br->Set(0.85 * -Deadband(driver->GetZ(), 0.08));

		}
		//rotation in opposite direction
		if (driver->GetZ() <= -0.08)
		{
			//rotate wheels to rotaionary position
			mix = true;
			if (!totecrot)
			{
				aPID(flFV, flS, tote_ANGLE_1, 1, i_fl);
				aPID(frFV, frS, tote_ANGLE_2, 2, i_fr);
				aPID(blFV, blS, tote_ANGLE_4, 3, i_bl);
				aPID(brFV, brS, tote_ANGLE_3, 4, i_br);
			}
			else
				if (!backcrot)
				{
					aPID(flFV, flS, back_ANGLE_1, 1, i_fl);
					aPID(frFV, frS, back_ANGLE_2, 2, i_fr);
					aPID(blFV, blS, back_ANGLE_4, 3, i_bl);
					aPID(brFV, brS, back_ANGLE_3, 4, i_br);
				}
				else
				{
					aPID(flFV, flS, ANGLE_1, 1, i_fl);
					aPID(frFV, frS, ANGLE_2, 2, i_fr);
					aPID(blFV, blS, ANGLE_4, 3, i_bl);
					aPID(brFV, brS, ANGLE_3, 4, i_br);
				}
			//set translational talon throttle
			fl->Set(0.85 * Deadband(driver->GetZ(), 0.08));
			fr->Set(0.85 * -Deadband(driver->GetZ(), 0.08));
			bl->Set(0.85 * Deadband(driver->GetZ(), 0.08));
			br->Set(0.85 * -Deadband(driver->GetZ(), 0.08));
		}

		if (driver->GetRawButton(1))
		{
			visionpushed = 1;
			switch (VisionState)
			{

				case visstrafeprep:
					printf("%i VisionState: \n", VisionState);
					RightEnc();
					aPID(flFV, flS, 90, 1, i_fl);
					aPID(frFV, frS, 90, 2, i_fr);
					aPID(blFV, blS, 90, 3, i_bl);
					aPID(brFV, brS, 90, 4, i_br);
					printf("%f aError \n", aError);
					if (Deadband(aError, 5) == 0)
					{
						VisionState = visstrafe;
						printf("%i VisionState: \n", VisionState);
					}
					break;
				case visstrafe:
					printf("%i VisionState: %f objx \n", VisionState, objx);
					flS->Set(0);
					blS->Set(0);
					frS->Set(0);
					brS->Set(0);

					fl->Set(-VisionPID(objx, objxPos, objxP, objxI) / 3);
					bl->Set(VisionPID(objx, objxPos, objxP, objxI) / 3);
					fr->Set(-VisionPID(objx, objxPos, objxP, objxI) / 3);
					br->Set(-VisionPID(objx, objxPos, objxP, objxI) / 3);
					if (Deadband((objx - objxPos), 2) == 0)
					{
						fl->Set(0);
						bl->Set(0);
						fr->Set(0);
						br->Set(0);
						VisionState = donedefault;
					}
					break;

				case donedefault:
					printf("%i VisionState: \n", VisionState);
					break;
			}
		}
		else
		{
			if (visionpushed == 1)
			{
				fl->Set(0);
				bl->Set(0);
				fr->Set(0);
				br->Set(0);
				flS->Set(0);
				blS->Set(0);
				frS->Set(0);
				brS->Set(0);
				VisionState = visstrafeprep;
			}
			visionpushed = 0;
		}

	}
	float VisionPID(float cogvar, float cogpos, float cogP, float cogI)
	{
		cogPIDTar = cogvar;
		cogPIDPos = cogpos;
		cogPIDError = cogPIDTar - cogPIDPos;
		cogPIDPout = cogPIDError * cogP;
		cogPIDIout = cogPIDError * cogI;
		cogPIDSpeed = cogPIDPout + cogPIDIout;
		return cogPIDSpeed;
	}
	//converts encoder ticks to degrees.
	float encMath(float encV)
	{
		return (encV - LOWEST_ENC) * 360.0 / (HIGHEST_ENC - LOWEST_ENC);
	}
	//master PID function for auton
	void aPID(float encPos, CANTalon *talon, float encTar, int vP, float &i)
	{
		//Sets variable states for everything
		bool pt, tp;
		bool rev;

		switch (vP)
		{
			case 1:
				pt = flPT;
				tp = flTP;
				rev = flRev;
				break;
			case 2:
				pt = frPT;
				tp = frTP;
				rev = frRev;
				break;
			case 3:
				pt = blPT;
				tp = blTP;
				rev = blRev;
				break;
			case 4:
				pt = brPT;
				tp = brTP;
				rev = brRev;
				break;
		}
		aError = encTar - encPos;
		//changes encoder values based on if one is rotating,translating and takes into account rev
		if (!mix)
		{
			if (encTar > 180)
			{
				encTar -= 360;
			}
			if (encPos > 180)
			{
				encPos -= 360;
			}
			if (rev)
			{
				if (encTar > 0)
				{
					encTar += 180;
				}
				else
				{
					encTar -= 180;
				}
			}
			if (encTar - encPos > 90 && encTar - encPos < 270)
			{
				rev = !rev;
				encTar = encTar - 180;
			}
			else
				if (encTar - encPos < -90 && encTar - encPos > -270)
				{
					rev = !rev;
					encTar = encTar + 180;
				}
			if (encTar < 0)
			{
				encTar += 360;
			}
			if (encPos < 0)
			{
				encPos += 360;
			}
		}
		aError = encTar - encPos;
		if (encPos > 270 && encPos < 360 && encTar > 0 && encTar < 90)
		{
			pt = true;
			aError = encTar + 360 - encPos;
		}
		else
			if (encTar > 270 && encTar < 360 && encPos > 0 && encPos < 90)
			{
				tp = true;
				aError = encTar - 360 - encPos;
			}
		//Determines target relative to position
		if (aError < 1 || aError > -1)
		{
			switch (vP)
			{
				case 1:
					tp = false;
					pt = false;
					break;
				case 2:
					tp = false;
					pt = false;

					break;
				case 3:
					tp = false;
					pt = false;
					break;
				case 4:
					tp = false;
					pt = false;
					break;
			}
		}
		//I for PID stuff
		i += PID_I * aError;
		if (aError < 1)
			i = 0;
		iPID(aError, talon, i);
		if (aError < 10)
			mag = 0;

		//again sets states
		switch (vP)
		{
			case 1:
				flPT = pt;
				flTP = tp;
				flRev = rev;
				break;
			case 2:
				frPT = pt;
				frTP = tp;
				frRev = rev;
				break;
			case 3:
				blPT = pt;
				blTP = tp;
				blRev = rev;
				break;
			case 4:
				brPT = pt;
				brTP = tp;
				brRev = rev;
				break;
		}

	}
	//PID for i cumulative
	void iPID(float error, CANTalon *talon, float &i)
	{
		float deadbandE = Deadband(error, 1);
		float pidDeadband = PID_P * deadbandE;
		float pid = Deadband(pidDeadband, 0.05);

		talon->Set(-.75 * pid - i);

		PID_P = MAX_PID_P * AV(1 - AV(pid));
	}

	//making the encoders (0-3722)
	double straighten(double encP, float straight)
	{
		if (encP <= straight)
		{
			encP = encP + (HIGHEST_ENC - straight);
		}
		else
		{
			encP = encP - (straight + 1 - LOWEST_ENC);
		}
		return encP;
	}

	//basic PID for forklift movement
	void forkliftPID()
	{
		fEnc = lift->GetEncPosition();
		fError = fEnc - fTar;
		fSpeed = Deadband(fError * PID_P_F, .05);
	}
	//basic PID for claw movement
	void clawPID()
	{
		cEnc = -claw->GetEncPosition();
		cError = cTar - cEnc;
		clawcumError += cError;
		if ((Deadband(cError, 20) == 0 || pdp->GetCurrent(9) >= 6) && clawPIDCheck == true)
		{
			cSpeed = 0;
			clawPIDCheck = false;
		}
		cSpeed = -1 * Deadband((cError * PID_P_C + clawcumError * PID_I_C), 0.08);
		claw->Set(cSpeed);
	}
	//limit switches on bottom hit move up so forklift doesnt break
	void forkliftLimit()
	{
		if ((dl->Get() == 0 || dr->Get() == 0))
		{
			lift->SetPosition(0);
			if (op->GetY() >= 0)
			{
				fSpeed = 0;
			}

		}
	}

	//determine direction of throttle
	void rev()
	{
		if (flRev)
		{
			flSpeed = flSpeed * -1;

		}
		if (frRev)
		{
			frSpeed = frSpeed * -1;
		}
		if (blRev)
		{
			blSpeed = blSpeed * -1;
		}
		if (brRev)
		{
			brSpeed = brSpeed * -1;
		}
	}

	//resets encoders
	void EncReset()
	{
		fl->SetPosition(0);
		fr->SetPosition(0);
		bl->SetPosition(0);
		br->SetPosition(0);

		while (fl->GetEncPosition() != 0)
		{
			fl->SetPosition(0);
		}
		while (fr->GetEncPosition() != 0)
		{
			fr->SetPosition(0);
		}
		while (bl->GetEncPosition() != 0)
		{
			bl->SetPosition(0);
		}
		while (br->GetEncPosition() != 0)
		{
			br->SetPosition(0);
		}

	}

	void TestInit()
	{
	}

	//used for straightening wheels: run test and replace corresponding straighten values
	void TestPeriodic()
	{
		flV = HIGHEST_ENC - (float) flSE->GetValue() + LOWEST_ENC;
		frV = HIGHEST_ENC - (float) frSE->GetValue() + LOWEST_ENC;
		blV = HIGHEST_ENC - (float) blSE->GetValue() + LOWEST_ENC;
		brV = HIGHEST_ENC - (float) brSE->GetValue() + LOWEST_ENC;

		printf("flV: %f frV: %f blV: %f brV: %f angle: %f\n", flV, frV, blV, brV, gyro->GetAngle());

	}

	//reset stuff when we disable
	void DisabledInit()
	{
		ResetEverything();
	}
	void DisabledPeriodic()
	{
		ResetEverything();
	}
	//look at the name lol
	void ResetEverything()
	{
		fl->Set(0);
		fr->Set(0);
		bl->Set(0);
		br->Set(0);
		flS->Set(0);
		frS->Set(0);
		brS->Set(0);
		blS->Set(0);
		lift->Set(0);
		claw->Set(0);
		time->Reset();
		PID_I_G = 0;
		Twocan = grab2C;
		OneCanAuton = closes;
		VisionState = visstrafeprep;
		clawcumError = 0;
		gyrocumError = 0;
		tcumError = 0;
		autonCheck = true;
		PID_I_C = 0;
		//setting all values to encoder for first time, then making the previous value the same
		clawPIDCheck = false;
		liftPIDCheck = false;
		gyro->Reset();
		EncReset();
		pTimer->Reset();
		PID_P_C = 0.02;
		flV = flSE->GetValue();
		frV = frSE->GetValue();
		blV = blSE->GetValue();
		brV = brSE->GetValue();

		flPT = false;
		flTP = false;
		frPT = false;
		frTP = false;
		blPT = false;
		blTP = false;
		brPT = false;
		brTP = false;

		claw->SetPosition(0);
		lift->SetPosition(0);
		fEnc = lift->GetPosition();
		cEnc = claw->GetPosition();
		fTar = fEnc;
		fEnc = 0;
		cTar = cEnc;
		fError = 0;
		cError = 0;

		flRev = false;
		frRev = false;
		blRev = false;
		brRev = false;

		mix = false;
		preset = false;

		PID_P_F = 0.00275;
		bounce = false;
		tcumError = 0;
	}
	void Snake()
	{
		//Measurements will need to be updated since the modules were moved. These are the distances
		//between the centerpoint of each wheel module, not the robot dimensions.
		float length = 30.5;
		float width = 20;

		float input = driver->GetZ();
		float pi = 3.141592653589793238462643383279502884197169399375105820974944592307816406286208998628034825342117067;
		float r_cp = 0;
		float a_i = 0;

		float a_cl = input * pi / 2;
		float r_cl = length / (2 * sin(a_cl));

		float radius = r_cl + width;

		if (input == -1 || input == 1)
			r_cp = 0;
		else
			if (Deadband(input, 0.08) < 0)
				r_cp = -1 * sqrt((r_cl * r_cl) - ((length * length) / 4));
			else
				r_cp = sqrt((r_cl * r_cl) - ((length * length) / 4));

		if (r_cp < (width / 2))
		{
			a_i = 3.14 - atan((length / 2) / (r_cp - (width / 2)));
		}
		else
			if (r_cp > (width / 2))
			{
				if (input < 0)
					a_i = -1 * atan((length / 2) / (r_cp - (width / 2)));
				else
					if (input > 0)
						a_i = atan((length / 2) / (r_cp - (width / 2)));
			}

		float a_o = atan((length / 2) / (r_cp + (width / 2)));

		float r_o = length / (2 * sin(a_o));
		float r_i = length / (2 * sin(a_i));

		if (input > 0)
		{
			fl->Set(Deadband(-driver->GetY() * (r_o / radius), 0.08));
			fr->Set(-Deadband(-driver->GetY() * (r_i / radius), 0.08));
			bl->Set(Deadband(-driver->GetY() * (r_o / radius), 0.08));
			br->Set(-Deadband(-driver->GetY() * (r_i / radius), 0.08));
			aPID(flFV, flS, a_o, 1, i_fl);
			aPID(frFV, frS, a_i, 2, i_fr);
			aPID(blFV, blS, -a_o, 3, i_bl);
			aPID(brFV, brS, -a_i, 4, i_br);
		}
		else
			if (input < 0)
			{
				fl->Set(Deadband(-driver->GetY() * (r_i / radius), 0.08));
				fr->Set(-Deadband(-driver->GetY() * (r_o / radius), 0.08));
				bl->Set(Deadband(-driver->GetY() * (r_i / radius), 0.08));
				br->Set(-Deadband(-driver->GetY() * (r_o / radius), 0.08));
				aPID(flFV, flS, a_i, 1, i_fl);
				aPID(frFV, frS, a_o, 2, i_fr);
				aPID(blFV, blS, -a_i, 3, i_bl);
				aPID(brFV, brS, -a_o, 4, i_br);
			}

	}

	//set getY to 0 if within sensitivity
	float Deadband(float getY, float sensitivity)
	{
		if (getY > sensitivity || getY < -sensitivity)
		{
			return getY;
		}
		else
		{
			return 0;
		}
		return getY;
	}

};

START_ROBOT_CLASS(Robot);
