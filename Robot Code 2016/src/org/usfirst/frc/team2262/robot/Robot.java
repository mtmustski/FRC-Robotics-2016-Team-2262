package org.usfirst.frc.team2262.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.CameraServer;
//import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	RobotDrive myRobot;

	/*
	 * // adding CAN Talons for drive motors
	 * 
	 * 
	 * driveMotorClass = new DriveMotor(front, back); DriveMotor leftDrive;
	 * DriveMotor rightDrive;
	 * 
	 * 
	 * 
	 * TalonSRX frontLeft; TalonSRX rearLeft; TalonSRX frontRight; TalonSRX
	 * rearRight;
	 * 
	 * 
	 * 
	 * adding victors for arm Talon elbow; Talon roller;
	 * 
	 * 
	 * 
	 * adding victors for Tape Measure Talon frictionWheel; Talon backClimber;
	 * Talon frontClimber;
	 * 
	 */

	// adding drive class
	Drive drive;

	// adding arm class
	Arm arm;

	// adding tape measure class
	TapeMeasure tapeMeasure;

	// adding encoder class
	WheelRotation encoder;

	// adding sensors

	Ultrasonic ultrasonic;

	DigitalInput limitSwitchTop;
	DigitalInput limitSwitchBottom;

	// declaring imu
	ADIS16448_IMU imu;
	int autoLoopCounter;

	// declaring encoder distancePerPulse
	double distancePerPulse;

	// declare smartdashboard
	SmartDashboard smartDashboard;

	// adding camera
	CameraServer server;

	/*
	 * adding joystick Joystick joystick;
	 */

	// adding xbox 360 controller
	Joystick controller;

	// adding xbox controller mapping
	ControllerMapping controllerMapping;

	// Autonomous variables.
	int distance = 20;
	int howFarToGo = 0;
	double imuAngleOriginal = 0;
	double imuAngleCurrent = 0;
	double desiredDegrees = 0;

	enum AutonomousState {
		MoveToX, TrunToTower, MoveToTower, Aim, Shoot, Done
	};

	AutonomousState myState = AutonomousState.MoveToX;
	double direction = 0.0;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	public void robotInit() {
		// initialize smartDashboard:
		smartDashboard = new SmartDashboard();

		// resetting encoders
		encoder = new WheelRotation(6, 360);

		// Initialing imu
		imu = new ADIS16448_IMU();

		// calibrating the imu DO NOT TOUCH ROBIT!!!
		imu.calibrate();

		// initializing drive class
		drive = new Drive(0, 1, 2, 3, 0);

		// initializing arm class
		arm = new Arm(0, 1, 8, 9);

		// initializing tape measure class
		tapeMeasure = new TapeMeasure(2, 3, 4);

		// creating new instance of xboxController
		controller = new Joystick(1);

		// creating new instance of controllerMapping
		controllerMapping = new ControllerMapping();

		/*
		 * creating new instance of joystick joystick = new Joystick(0);
		 */

		// camera
		server = CameraServer.getInstance();
		server.setQuality(50);
		// the camera name (ex "cam0") can be found through the roborio web
		// interface
		server.startAutomaticCapture("cam0");

		// initializing drive motors

		/*
		 * leftDrive = new DriveMotor(0, 1); rightDrive = new DriveMotor(2, 3);
		 */

		/*
		 * frontLeft = new TalonSRX(0); rearLeft = new TalonSRX(1); frontRight =
		 * new TalonSRX(2); rearRight = new TalonSRX(3);
		 *
		 * initializing RobotDrive myRobot = new RobotDrive(frontLeft, rearLeft,
		 * frontRight, rearRight);
		 */

		/*
		 * //initializing arm motors elbow = new Talon(0); armRoller = new
		 * Talon(1);
		 */

	}

	/**
	 * This function is run once each time the robot enters autonomous mode
	 */
	public void autonomousInit() {
		autoLoopCounter = 0;
		distance = 20;
		howFarToGo = 0;

		// displaying the original angle of the imu on the dash
		double imuAngleFirst = imu.getAngle();
		SmartDashboard.putNumber("Imu Angle Original", imuAngleFirst);

		// angle we need to turn to for tower
		desiredDegrees = 30;
	}

	/**
	 * This function is called periodically during autonomous
	 */
	public void autonomousPeriodic() {
		/*
		 * if (autoLoopCounter < 100) // Check if we've completed 100 loops //
		 * (approximately 2 seconds) { myRobot.drive(-0.5, 0.0); // drive
		 * forwards half speed autoLoopCounter++; } else { myRobot.drive(0.0,
		 * 0.0); // stop robot }
		 */

		imuAngleCurrent = imu.getAngle();
		double imuAngleDiffrence = imuAngleCurrent - imuAngleOriginal;

		switch (myState) {
		case MoveToX:

			if (encoder.getDistance() > 10) {
				drive.stop();

				myState = AutonomousState.TrunToTower;

			} else {
				if (imuAngleDiffrence > 0) {
					drive.turnLeft(0.2);
				} else if (imuAngleDiffrence < 0) {
					drive.turnRight(.2);
				} else
					drive.driveForward(.5);
			}

			// Have I turned?
			// How far have I gone?
			// How far do I have to go?
			// have I got there?
			// If yes. myState = TurnToTower;

			break;
		case TrunToTower:

			if (desiredDegrees == imuAngleCurrent) {
				drive.stop();
				myState = AutonomousState.MoveToTower;
			} else {
				if (imuAngleCurrent < desiredDegrees)
					drive.turnRight(0.2);
			}
			if (imuAngleCurrent > desiredDegrees) {
				drive.turnLeft(.2);
			}

			// Have I turned enough?
			// If yes. myState = MoveToTower

			break;
		case MoveToTower:

			// Have I turned?
			// How far have I gone?
			// How far do I have to go?
			// Have I got there.
			// If yes. myState = Aim

			break;
		case Aim:
			// Aim correct?
			// If yes. myState = shoot;
			// If no.
			// Need to trun left? if so turn
			// Need to Turn right? if so turn

			break;
		case Shoot:
			// Shoot;
			myState = AutonomousState.Done;

		case Done:
			// flash lights.
			// Sound horn.
		}

	}

	/**
	 * This function is called once each time the robot enters tele-operated
	 * mode
	 */
	public void teleopInit() {

		/*
		 * leftDrive.stop(); rightDrive.stop();
		 */

	}

	/**
	 * This function is called periodically during operator control
	 */
	public void teleopPeriodic() {

		// drive
		drive.driveMotion();

		// arm
		arm.ballIntake(controller.getRawButton(controllerMapping.leftBumper));
		arm.ballOutput(controller.getRawButton(controllerMapping.rightBumper));
		arm.elbowMotion(controller.getRawAxis(controllerMapping.triggers));
		arm.rollerMotion(controller.getRawButton(controllerMapping.buttonX),
				controller.getRawButton(controllerMapping.buttonB));

		// tape measure
		tapeMeasure.pushUp(controller.getRawButton(controllerMapping.buttonY));
		tapeMeasure.pullDown(controller.getRawButton(controllerMapping.buttonA));
	}

	/**
	 * This function is called periodically during test mode
	 */
	public void testPeriodic() {
		LiveWindow.run();

		// drive
		drive.driveMotion();

		// arm
		arm.ballIntake(controller.getRawButton(controllerMapping.leftBumper));
		arm.ballOutput(controller.getRawButton(controllerMapping.rightBumper));
		arm.elbowMotion(controller.getRawAxis(controllerMapping.triggers));
		arm.rollerMotion(controller.getRawButton(controllerMapping.buttonX),
				controller.getRawButton(controllerMapping.buttonB));

		// tape measure
		tapeMeasure.pushUp(controller.getRawButton(controllerMapping.buttonY));
		tapeMeasure.pullDown(controller.getRawButton(controllerMapping.buttonA));
		
		//smart dashboard diagnostics
		smartDashboard.putNumber("Front Left PWM", drive.frontLeft.get());
		smartDashboard.putNumber("Rear Left PWM", drive.rearLeft.get());
		smartDashboard.putNumber("Front Right PWM", drive.frontRight.get());
		smartDashboard.putNumber("Rear Right PWM", drive.rearRight.get());

	}

}