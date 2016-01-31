package org.usfirst.frc.team2262.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.TalonSRX;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.DigitalInput;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	RobotDrive myRobot;
	
	//adding CAN Talons for drive motors
	
	/*driveMotorClass = new DriveMotor(front, back);
	DriveMotor leftDrive;
	DriveMotor rightDrive;*/
	
	TalonSRX frontLeft;
	TalonSRX rearLeft;
	TalonSRX frontRight;
	TalonSRX rearRight;
	
	//adding victors for arm
	Talon elbow; 
	Talon armRoller;
	
	//adding victors for Tape Measure
	Talon tapeRoller;
	Talon frontClimber;
	Talon backClimber; 
	
	//adding sensors
	Encoder leftEncoder;
	Encoder rightEncoder;
	
	Ultrasonic ultrasonic;
	
	DigitalInput limitSwitchTop;
	DigitalInput limitSwitchBottom;
	
	int autoLoopCounter;
	
	//adding camera 
	CameraServer server; 
	
	//adding joystick
	Joystick joystick;
	
	//adding xbox 360 controller
	Joystick xboxController;
	
	//adding arm class
	Arm arm;
		
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */	
    public void robotInit() {
    	
    	//creating new instance of xboxController
    	xboxController = new Joystick(1);
    	
    	//creating new instance of joystick
    	joystick = new Joystick(0);
    	
    	//camera 
    	server = CameraServer.getInstance();
        server.setQuality(50);
        //the camera name (ex "cam0") can be found through the roborio web interface
        server.startAutomaticCapture("cam0");
     
        //initializing drive motors 
        
        /*leftDrive = new DriveMotor(0, 1);
        rightDrive = new DriveMotor(2, 3);*/
        
        frontLeft = new TalonSRX(0);
        rearLeft = new TalonSRX(1);
        frontRight = new TalonSRX(2);
        rearRight = new TalonSRX(3);
        
        //initializing RobotDrive
        myRobot = new RobotDrive (frontLeft, rearLeft, frontRight, rearRight);
        
       /* //initializing arm motors
        elbow = new Talon(0);
        armRoller = new Talon(1); */
        
        //initializing arm class
        arm = new Arm(0, 1, 8, 9);    
        
    }
    
    /**
     * This function is run once each time the robot enters autonomous mode
     */
    public void autonomousInit() {
    	autoLoopCounter = 0;
    	
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
    	if(autoLoopCounter < 100) //Check if we've completed 100 loops (approximately 2 seconds)
		{
			myRobot.drive(-0.5, 0.0); 	// drive forwards half speed
			autoLoopCounter++;
			} else {
			myRobot.drive(0.0, 0.0); 	// stop robot
		}
    	
    }
    
    /**
     * This function is called once each time the robot enters tele-operated mode
     */
    public void teleopInit(){
    	
    	/*leftDrive.stop();
    	rightDrive.stop();*/  
    	
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
    	//drive
    	myRobot.arcadeDrive(xboxController, 5, xboxController, 1);
    	
    	//arm
    	arm.elbowMotion (xboxController.getRawButton(6), xboxController.getRawButton(5));
    	
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    	LiveWindow.run();
    	
    }
    
}
