#include "WPILib.h"

class TestProgram: public IterativeRobot {
	Victor *left;
	Joystick *driver;
public:
	TestProgram() {
		left = new Victor(2);
		driver = new Joystick(1);
	}

	void TeleopInit() {

	}

	void TeleopPeriodic() {
		float speed = deadband(-driver->GetY());
		left->Set(speed);
	}

	void TestPeriodic() {
		left->Set(.4);
	}

	float deadband(float input) {
		if (input > 0.08 || input < -0.08) {
			return input;
		}
		return 0;
	}

};

START_ROBOT_CLASS (TestProgram);

