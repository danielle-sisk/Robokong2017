#include "WPILib.h"
#include <stdio.h>
#include <Spark.h>
#include <CANSpeedController.h>
//#include <SendableChooser.h>
//#include <LiveWindow.h>
//#include "LiveWindow/LiveWindowSendable.h"
#include <Talon.h>
#include "SafePWM.h"
#include "SpeedController.h"
#include "PWMSpeedController.h"
#include "Commands/Scheduler.h"
#include "LiveWindow/LiveWindowSendable.h"
#include "tables/ITable.h"
#include <memory>
#include <string>
#include "SmartDashboard/SendableChooserBase.h"
#include "llvm/StringMap.h"
#include "llvm/StringRef.h"
#include "tables/ITable.h"
#include "Base.h"
#include "ErrorBase.h"

struct GCODE_STRUCT
{
 float Left;
 float Right;
 double Time;
} gcode_st;

class Robot: public IterativeRobot
{
private:
	LiveWindow *lw = LiveWindow::GetInstance();
	frc::SendableChooser<std::string> *chooser;
	//const std::string autoNameDefault = "Option 1";
	const std::string autoNameDefault = "Option 1";
	const std::string autoNameCustom2 = "Playback Mode";
	const std::string autoNameCustom3 = "Option 3";
	const int HH_MAX_DOWN = 14700;
	std::string autoSelected;

	//Command *m_AutonomousCommand;
	//Notifier *m_EventNotifier;
	GCODE_STRUCT m_gc;					// GCODE Block
	FILE *m_pGCODEFile;					// GCODE file handle
  	float m_fHoldLeft, m_fHoldRight;	// Joystick Hold positions
	int m_iCounter;						// timer elapsed seconds
	int m_iHHSensorPosition;			// rotary encoder clicks - 0=home limit switch
	bool  m_bTimerRunning;				// true when timer is running
	bool  m_bFirstAutonomous;			// 1st execution of AutonomousPeriodic
	Timer *m_ElapsedTimer;				// Elapssed Time in seconds
	Timer *m_GCODETimer;
	Talon *m_motorLeft;      		// #1 - #2 Follows #1
	Talon *m_motorLeftSlave;
	Talon *m_motorRight;     		// #4 - #3 Follows #3
	Talon *m_motorRightSlave;
	Talon *m_motorTrigger;   		// Ball Trigger window motor
	Talon *m_motorLauncher;  		// Ball launch wheel
	TalonSRX *m_motorHHDrive;   		// HammerHead drive, Snow Blower
	Joystick *m_stickLeft;      		// left tank drive stick
	Joystick *m_stickRight;     		// right tank dirive stick
	Joystick *m_stickShooter;   		// ball shooter stick
	DigitalInput *m_ShooterSwitch;  	// tells us 'load' position
	DigitalInput *m_HHHomeSwitch;  		// tells us 'hammer head home' position
	//RobotDrive m_drive = new RobotDrive(m_motorLeft, m_motorRight);

	void RobotInit()
	{
//		chooser = new SendableChooser();
//		const string {aka const std::basic_string<char>}
		//chooser->frc::SendableChooser::AddDefault(autoNameDefault, (void*)autoNameDefault);
		chooser->SendableChooser::AddDefault(autoNameDefault, autoNameDefault);
		chooser->SendableChooser::AddObject(autoNameCustom2, autoNameCustom2);
		chooser->SendableChooser::AddObject(autoNameCustom3, autoNameCustom3);
		SmartDashboard::PutData("Auto Modes", chooser);

//		m_AutonomousCommand = new ACBreachDefense();    // autonomous command
//		m_EventNotifier = new Notifier((TimerEventHandler*)&evt_Notifier);
		m_fHoldLeft = 0.0;
		m_fHoldRight = 0.0;
		m_iHHSensorPosition = 0;
		m_iCounter = 0;
		m_bTimerRunning = false;
		m_ElapsedTimer = new Timer();
		m_GCODETimer = new Timer();
		m_motorLeft = new Talon(1);      // #1 - #2 Follows #1
		m_motorLeftSlave = new Talon(2);
		m_motorRight = new Talon(4);     // #4 - #3 Follows #3
		m_motorRightSlave = new Talon(3);
		m_motorTrigger = new Talon(6);   // Ball Trigger window motor
		m_motorLauncher = new Talon(7);  // Ball launch wheel
		m_motorHHDrive = new TalonSRX(5);   // HammerHead drive, Snow Blower
		m_stickLeft = new Joystick(0);      // left tank drive stick
		m_stickRight = new Joystick(1);     // right tank dirive stick
		m_stickShooter = new Joystick(2);   // ball shooter stick
		m_ShooterSwitch = new DigitalInput(0);  // tells us 'load' position
		m_HHHomeSwitch =  new DigitalInput(1);  // tells us 'hammer head home' position
		m_bFirstAutonomous = true;

//		setup the follower CANTalons for the drivebase
//		m_motorLeftSlave->SetControlMode(frc::Talon);//::kFollower);
		//m_motorLeftSlave->SetControlMode(Talon::kDigitalChannels)
		m_motorLeftSlave->SetRaw(Talon::kDigitalChannels);
		m_motorLeftSlave->Set(3);
		m_motorRightSlave->SetRaw(Talon::kDigitalChannels);
		m_motorRightSlave->Set(2);
		m_ElapsedTimer->Stop();

		//m_motorHHDrive->SetSensorPosition(m_iHHSensorPosition);
		//m_motorHHDrive->SetFeedbackDeviceSelect(TalonSRX::kFeedbackDev_DigitalQuadEnc);		// quad encoder 0
		m_motorHHDrive->SensorBase(m_iHHSensorPosition);
		m_motorHHDrive->SetRaw(TalonSRX::)

//		live window HammerHead group
//		lw->AddActuator("HammerHead", "Drive", m_motorHHDrive);
		lw->AddActuator("HammerHead", "Trigger", m_motorTrigger);
		lw->AddActuator("HammerHead", "Launcher", m_motorLauncher);
		lw->AddSensor("HammerHead", "TiggerSW", m_ShooterSwitch);
		lw->AddSensor("HammerHead", "HomeSW", m_HHHomeSwitch);

        // live window drive train group
		lw->AddActuator("DriveTrain", "left master", m_motorLeft);
		lw->AddActuator("DriveTrain", "left slave", m_motorLeftSlave);
		lw->AddActuator("DriveTrain", "right master", m_motorRight);
		lw->AddActuator("DriveTrain", "right slave", m_motorRightSlave);

		SmartDashboard::PutNumber("SensorPos", m_iHHSensorPosition);
		//SmartDashboard::PutNumber("stickLeft", 0.0);
		//SmartDashboard::PutNumber("stickRight", 0.0);
	}


	/**
	 * This autonomous (along with the chooser code above) shows how to select between different autonomous modes
	 * using the dashboard. The sendable chooser code works with the Java SmartDashboard. If you prefer the LabVIEW
	 * Dashboard, remove all of the chooser code and uncomment the GetString line to get the auto name from the text box
	 * below the Gyro
	 *
	 * You can add additional auto modes by adding additional comparisons to the if-else structure below with additional strings.
	 * If using the SendableChooser make sure to add them to the chooser code above as well.
	 */
	void AutonomousInit()
	{
		//autoSelected = *((std::string*)chooser->GetSelected());
		autoSelected = (chooser->GetSelected());
		//std::string autoSelected = SmartDashboard::GetString("Auto Selector", autoNameDefault);
		std::cout << "Auto selected: " << autoSelected << std::endl;

		if(autoSelected == autoNameCustom2)  // playback
		{
		 m_bFirstAutonomous = true;
		 open_gcode(1);  // open gcode file to read
		}
		else
		{
			//if(m_AutonomousCommand != NULL)
			//{
			// m_AutonomousCommand->Start();
			//}
			//Default Auto goes here
		}
	}

	void AutonomousPeriodic()
	{
	 if(autoSelected == autoNameCustom2)
	 {
      if(m_bFirstAutonomous)
	   read_gcode(&m_gc);
      //----------------------------------------------------------
      //SmartDashboard::PutNumber("GC-Timer", m_GCODETimer->Get());
      //SmartDashboard::PutNumber("gc.Left", m_gc.Left);
      //SmartDashboard::PutNumber("gc.Right", m_gc.Right);
      //SmartDashboard::PutNumber("gc.Time", m_gc.Time);
      //----------------------------------------------------------
      if(m_GCODETimer->Get() >= m_gc.Time)
      {
       m_motorLeft->Set(m_gc.Left);
       m_motorRight->Set(m_gc.Right);
 	   read_gcode(&m_gc);
      }
      m_bFirstAutonomous = false;
	 }
	 else
	 {
			//Default Auto goes here
	 }
	 //Scheduler::GetInstance()->Run();
	}

	void TeleopInit()
	{
     open_gcode(0);  // open gcode file to write
	}

	void TeleopPeriodic()
	{
     // -----------------------------------------------------------------------------
     // write the joystick positions to the gcode file, if changed
	 // -----------------------------------------------------------------------------
	 write_gcode();   // write joystick values to gcode file
	 // -----------------------------------------------------------------------------
	 m_motorLeft->Set(m_stickLeft->GetY() * -1.0);
	 m_motorRight->Set(m_stickRight->GetY() * -1.0);
	 // -----------------------------------------------------------------------------
	 SmartDashboard::PutNumber("stickLeft", m_stickLeft->GetY() * -1.0);
	 SmartDashboard::PutNumber("stickRight", m_stickRight->GetY() * -1.0);
	 // -----------------------------------------------------------------------------
     // update the joystick hold positions
	 // -----------------------------------------------------------------------------
	 m_fHoldLeft = m_stickLeft->GetY() * -1.0;
     m_fHoldRight = m_stickRight->GetY() * -1.0;
     // -----------------------------------------------------------------------------
	 SmartDashboard::PutBoolean("HHHome", m_HHHomeSwitch->Get());

	 if(!m_HHHomeSwitch->Get())
	 {
	  m_motorHHDrive->Set(0.0);
	  m_iHHSensorPosition = 0;
	  m_motorHHDrive->SetSensorPosition(m_iHHSensorPosition);
	 }

	 m_motorHHDrive->GetSensorPosition(m_iHHSensorPosition);
	 m_iHHSensorPosition = m_iHHSensorPosition * -1;
	 SmartDashboard::PutNumber("SensorPos", m_iHHSensorPosition);

     // -----------------------------------------
     // lift hammerhead
	 if( (m_stickShooter->GetY() > 0.2) && (m_HHHomeSwitch->Get()) )
	 {
	  m_motorHHDrive->Set(-1.0);
	 }
     // -----------------------------------------

	 // lower hammerhead
	 if((m_stickShooter->GetY() < -0.2) && (m_iHHSensorPosition < HH_MAX_DOWN))
	 {
	  m_motorHHDrive->Set(1.0);
	 }
     // -----------------------------------------

	 if(fabs(m_stickShooter->GetY()) < 0.2)
	 {
	  m_motorHHDrive->Set(0.0);
	 }

	 if((m_iHHSensorPosition >= HH_MAX_DOWN) && (m_stickShooter->GetY() < 0.2))
	 {
	  m_motorHHDrive->Set(0.0);
	 }
     // -----------------------------------------

	 // ball shooter trigger pulled - Joystick(1) - button #1
	 if(m_stickShooter->GetRawButton(1))
	 {
	  //m_motorTrigger->Set(1.0);   // start trigger motor
	  m_motorLauncher->Set(-1.0); // start ball launcher motor

	  if(!m_bTimerRunning)
	  {
	   m_ElapsedTimer->Start();
	   m_ElapsedTimer->Reset();
	   m_bTimerRunning = true;
	  }

	  if(m_ElapsedTimer->Get() > 1.3)		// 1.25s start ball shooter
	   m_motorTrigger->Set(1.0);   // start trigger motor
	 }
	 else
	 {
	  m_ElapsedTimer->Stop();
	  m_bTimerRunning = false;
	  if(!m_stickShooter->GetRawButton(2))
	   m_motorLauncher->Set(0.0); //stop ball launcher motor

	  if(!m_ShooterSwitch->Get())	// digital input state
	   m_motorTrigger->Set(0.0);  // stop trigger motor
	 }


	 if(m_stickShooter->GetRawButton(2)) 	// ball load
	  m_motorLauncher->Set(0.72);
	 else
	 {
	  if(!m_stickShooter->GetRawButton(1))
	   m_motorLauncher->Set(0.0);
	 }

    }

	void TestPeriodic()
	{
	 lw->Run();
	}

	void DisabledPeriodic()
	{
	 close_gcode();		// close the gcode file
	}

	// -----------------------------------------------------------------------------
    // function  : evt_notifier
    // purpose   : called when event fires @ regular intervals
	// -----------------------------------------------------------------------------
	void evt_Notifier()
	{
	 m_iCounter++;
	 SmartDashboard::PutNumber("evt_Notifier", m_iCounter);
	}

	// -----------------------------------------------------------------------------
    // function  : open_gcode
	// parameter : read=1, write=0
    // purpose   : open the gcode file in the specified mode
	// intention : teleop will write joystick positions, autonomous will read gcode file & execute
	// -----------------------------------------------------------------------------
    void open_gcode(int mode)
    {
     //    home/lvuser   can write to this folder
     close_gcode();

     if(mode == 0)
      m_pGCODEFile = fopen("home/lvuser/gcode.txt" , "w");
     else
      m_pGCODEFile = fopen("home/lvuser/gcode.txt" , "r");

     //if (m_pGCODEFile == NULL)
     // SmartDashboard::PutNumber("GCODE-FILE", 120);

     m_GCODETimer->Start();
    }

	// -----------------------------------------------------------------------------
    // function  : close_gcode
    // purpose   : close the gcode file
	// -----------------------------------------------------------------------------
    void close_gcode()
    {
     if (m_pGCODEFile != NULL)
      fclose(m_pGCODEFile);

     m_GCODETimer->Stop();
    }

    // -----------------------------------------------------------------------------
    // function  : read_gcode
    // purpose   : read left, right joystick positions from gcode file
	// -----------------------------------------------------------------------------
    void read_gcode(GCODE_STRUCT *gc)
    {
     char stmp[120] = "";
     char *pch;
     int c = 0;

     if (m_pGCODEFile == NULL)
      open_gcode(1);

     if (m_pGCODEFile != NULL)
     {
      if(fgets(stmp, 100, m_pGCODEFile) == NULL)
      {
       SmartDashboard::PutString("GCOCE", "EOF");
      }
      else
      {
       //SmartDashboard::PutString("gc read", stmp);
   	   pch = strtok(stmp, ",");
   	   while(pch != NULL)
   	   {
       //SmartDashboard::PutString("token", pch);
       switch(c)
       {
        case 0:
         gc->Left = atof(pch);
       	 break;
        case 1:
         gc->Right = atof(pch);
       	 break;
        case 2:
         gc->Time = atof(pch);
       	 break;
        }
        c++;
	    pch = strtok(NULL, ",");
   	   }
      	//SmartDashboard::PutNumber("gc->Left", gc->Left);
       	//SmartDashboard::PutNumber("gc->Right", gc->Right);
       	//SmartDashboard::PutNumber("gc->Time", gc->Time);
      }
     }
    }

    // -----------------------------------------------------------------------------
    // function  : write_gcode
    // purpose   : write left, right joystick positions togcode file
	// -----------------------------------------------------------------------------
    void write_gcode()
    {
     char stmp[100] = "";
     if (m_pGCODEFile != NULL)
     {
      if( (nearly(m_fHoldLeft, m_stickLeft->GetY() * -1.0)) || (nearly(m_fHoldRight, m_stickRight->GetY() * -1.0)) )
      {
       sprintf(stmp, "%f,%f,%f\n", m_stickLeft->GetY() * -1.0, m_stickRight->GetY() * -1.0, m_GCODETimer->Get());
       fputs(stmp, m_pGCODEFile);
      }
     }
    }

	// -----------------------------------------------------------------------------
    // function  : nearly
    // purpose   : test 2 floating point numbers & determine if they are different
    //           : by the tolerance amount.
	// -----------------------------------------------------------------------------
    bool nearly(float x, float y)
    {
     if(fabs(x - y) > 0.01)
   	  return true;
     else
      return false;
    }

};

START_ROBOT_CLASS(Robot)
