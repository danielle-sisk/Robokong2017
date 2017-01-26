#include <iostream>
#include <memory>
#include <string>

#include <Joystick.h>
#include <SampleRobot.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <RobotDrive.h>
#include <Timer.h>
#include <encoder.h>
#include <DriverStation.h>
#include <CounterBase.h>


/**
 * This is a demo program showing the use of the RobotDrive class.
 * The SampleRobot class is the base of a robot application that will
 * automatically call your Autonomous and OperatorControl methods at the right
 * time as controlled by the switches on the driver station or the field
 * controls.
 *
 * WARNING: While it may look like a good choice to use for your code if you're
 * inexperienced, don't. Unless you know what you are doing, complex code will
 * be much more difficult under this system. Use IterativeRobot or Command-Based
 * instead if you're new.
 */
class Robot: public frc::SampleRobot {

	bool squaredInputs = 0;
	frc::RobotDrive myRobot { 3, 2, 1, 0 }; // robot drive system
	frc::Joystick stickL {0}; // only joystick
	frc::Joystick stickR {1};
	frc::Joystick gamePad {0};
	frc::SendableChooser<std::string> chooser;
	const std::string autoNameDefault = "Default";
	const std::string autoNameCustom = "My Auto";
	frc::Encoder *E0;
	frc::DriverStation *ds;




public:
	Robot() {
		//Note SmartDashboard is not initialized here,
		//wait until RobotInit to make SmartDashboard calls
		ds = &frc::DriverStation::GetInstance();
		myRobot.SetExpiration(0.1);
		E0 = new Encoder(0,1,true,Encoder::EncodingType::k4X);
		E0->SetMaxPeriod(.1);
		E0->SetMinRate(10);
		E0->SetDistancePerPulse(5);
		E0->SetReverseDirection(false);
		E0->SetSamplesToAverage(7);
		E0->Reset();
	}

	void RobotInit() {
		chooser.AddDefault(autoNameDefault, autoNameDefault);
		chooser.AddObject(autoNameCustom, autoNameCustom);
		frc::SmartDashboard::PutData("Auto Modes", &chooser);
		E0->Reset();
	}

	/*
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * GetString line to get the auto name from the text box below the Gyro.
	 *
	 * You can add additional auto modes by adding additional comparisons to the
	 * if-else structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	void Autonomous() {
		auto autoSelected = chooser.GetSelected();
		// std::string autoSelected = frc::SmartDashboard::GetString("Auto Selector", autoNameDefault);
		std::cout << "Auto selected: " << autoSelected << std::endl;

		if (autoSelected == autoNameCustom) {
			// Custom Auto goes here
			std::cout << "Running custom Autonomous" << std::endl;
			myRobot.SetSafetyEnabled(false);
			myRobot.Drive(-0.25, 0.5); // spin at half speed
			frc::Wait(2.0);                // for 2 seconds
			myRobot.Drive(0.0, 0.0);  // stop robot
		} else {
			// Default Auto goes here
			std::cout << "Running default Autonomous" << std::endl;
			myRobot.SetSafetyEnabled(false);
			myRobot.Drive(-0.25, 0.5); // drive forwards half speed
			frc::Wait(2.0);                // for 2 seconds
			myRobot.Drive(0.0, 0.0);  // stop robot
		}
	}


	 //Runs the motors with arcade steering.

	void OperatorControl() override {
		char str[1000];
		myRobot.SetSafetyEnabled(true);
		while (IsOperatorControl() && IsEnabled()) {
			double d1;
			d1=E0->GetRate();
			SmartDashboard::PutNumber("Rate", d1);
			SmartDashboard::PutNumber("Distance", E0->GetDistance());
			SmartDashboard::PutBoolean("Alive", true);
			std::sprintf(str, "%d, \n", d1);  //sprintf(str, "%d. \n", d1);
						DriverStation::ReportError(str);

			// drive with arcade style (use right stick)
			//myRobot.TankDrive(stickR, stickL, 1);
			myRobot.TankDrive(gamePad, 5, gamePad, 1);

			// wait for a motor update time
			frc::Wait(0.005);
		}
	}

	 //Runs during test mode

	void Test() override {

	}
};

START_ROBOT_CLASS(Robot)
