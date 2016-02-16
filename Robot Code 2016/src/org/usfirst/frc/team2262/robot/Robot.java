package org.usfirst.frc.team2262.robot;

import edu.wpi.first.wpilibj.IterativeRobot;

//import adis16448.frc.ADIS16448_IMU;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
//import edu.wpi.first.wpilibj.CameraServer;
//import edu.wpi.first.wpilibj.Encoder;
//import edu.wpi.first.wpilibj.Ultrasonic;
//import edu.wpi.first.wpilibj.DigitalInput;
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

	Drive drive;

	ADIS16448_IMU imu;
	int autoLoopCounter;
	Arm arm;

	Joystick controller;

	ControllerMapping controllerMapping;

	TapeMeasure tapeMeasure;

	WheelRotation encoder;

	// Autonomous variables.
	int distance = 20;
	int howFarToGo = 0;
	double imuAngleOriginal = 0;
	double imuAngleCurrent = 0;
	double desiredDegrees = 0;
	long armLowerTimer = 0;
	double timerStart = 0;
	double elapsedTime = 0;
	

	enum AutonomousState {
		LowerArm, MoveToX, TurnToTower, MoveToTower, Aim, Shoot, Done
	};

	AutonomousState myState = AutonomousState.MoveToX;
	double direction = 0.0;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	public void robotInit() {
		// initialize smartDashboard:
		drive = new Drive(0, 1, 2, 3, 0);

		imu = new ADIS16448_IMU();

		imu.calibrate();

		arm = new Arm(0, 1, 8, 9);

		controller = new Joystick(1);

		controllerMapping = new ControllerMapping();

		tapeMeasure = new TapeMeasure(2, 3, 4);

		encoder = new WheelRotation(6, 360);

		// declaring encoder distancePerPulse
		double distancePerPulse;
	}

	/**
	 * This function is run once each time the robot enters autonomous mode
	 */
	public void autonomousInit() {
		
		autoLoopCounter = 0;
		distance = 20;
		howFarToGo = 0;
		armLowerTimer = 0;
		
		elapsedTime = 0;
		timerStart = Timer.getFPGATimestamp(); 
		
		
		// displaying the original angle of the imu on the dash
		double imuAngleFirst = imu.getAngle();
		SmartDashboard.putNumber("Imu Angle Original", imuAngleFirst);

		// angle we need to turn to for tower
		desiredDegrees = 60;
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
		
		elapsedTime = Timer.getFPGATimestamp() - timerStart; 
		
		switch (myState) {
		case LowerArm:
	
			if(elapsedTime < .5){
				arm.elbowMotion(.2, 0);
			}else
				myState = AutonomousState.MoveToX;
				
			break;
		case MoveToX:

			if (encoder.getDistance() > 220.75) {
				drive.stop();

				myState = AutonomousState.TurnToTower;

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
		case TurnToTower:

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
			timerStart = Timer.getFPGATimestamp(); 
			break;
		case Shoot:
			
			
			// Shoot;
			
			if(elapsedTime < .3){
				arm.ballOutput(true);	
			}
			
			myState = AutonomousState.Done;
			break;
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

		drive.driveMotion();

		// arm code
		boolean leftBumper = controller.getRawButton(controllerMapping.leftBumper);
		boolean rightBumper = controller.getRawButton(controllerMapping.rightBumper);
		double leftTrigger = controller.getRawAxis(controllerMapping.leftTrigger);
		double rightTrigger = controller.getRawAxis(controllerMapping.rightTrigger);
		boolean buttonX = controller.getRawButton(controllerMapping.buttonX);
		boolean buttonB = controller.getRawButton(controllerMapping.buttonB);

		if (leftBumper == true) {
			arm.ballIntake(leftBumper);
		}

		if (rightBumper == true) {
			arm.ballOutput(rightBumper);
		}

		if (leftTrigger != 0.0 || rightTrigger != 0.0) {
			arm.elbowMotion(leftTrigger, rightTrigger);
		}

		if (buttonX == true || buttonB == true) {
			arm.rollerMotion(buttonX, buttonB);
		}

		if (!(leftBumper || rightBumper || leftTrigger != 0 || rightTrigger != 0 || buttonX || buttonB)) {
			arm.elbow.set(0);
			arm.roller.set(0);
		}

		// arm.ballIntake(controller.getRawButton(controllerMapping.leftBumper));
		// arm.ballOutput(controller.getRawButton(controllerMapping.rightBumper));
		// arm.elbowMotion(controller.getRawAxis(controllerMapping.triggers));
		// arm.rollerMotion(controller.getRawButton(controllerMapping.buttonX),
		// controller.getRawButton(controllerMapping.buttonB));

		// tape measure code
		boolean buttonY = controller.getRawButton(controllerMapping.buttonY);
		boolean buttonA = controller.getRawButton(controllerMapping.buttonA);

		if (buttonY) {
			tapeMeasure.pushUp(buttonY);
		}

		if (buttonA) {
			tapeMeasure.pullDown(buttonA);
		}

		if (!(buttonY || buttonX)) {
			tapeMeasure.frictionWheel.set(0);
			tapeMeasure.frontClimber.set(0);
			tapeMeasure.rearClimber.set(0);
		}

		imu.getAngle();

		tapeMeasure.pushUp(controller.getRawButton(controllerMapping.buttonY));
		tapeMeasure.pullDown(controller.getRawButton(controllerMapping.buttonA));

		SmartDashboard.putNumber("IMU X Angle", imu.getAngleX());
		SmartDashboard.putNumber("IMU Y Angle", imu.getAngleY());
		SmartDashboard.putNumber("IMU Z Angle", imu.getAngleZ());

		SmartDashboard.putNumber("Front Left CAN", drive.frontLeft.get());
		SmartDashboard.putNumber("Rear Left CAN", drive.rearLeft.get());
		SmartDashboard.putNumber("Front Right CAN", drive.frontRight.get());
		SmartDashboard.putNumber("Rear Right CAN", drive.rearRight.get());

		SmartDashboard.putNumber("Elbow PWM", arm.elbow.get());
		SmartDashboard.putNumber("Roller PWM", arm.roller.get());

		SmartDashboard.putNumber("Friction Wheel PWM", tapeMeasure.frictionWheel.get());
		SmartDashboard.putNumber("Front Climber PWM", tapeMeasure.frontClimber.get());
		SmartDashboard.putNumber("Rear Climber PWM", tapeMeasure.rearClimber.get());

		SmartDashboard.putNumber("Left Trigger Value", controller.getRawAxis(controllerMapping.leftTrigger));
		SmartDashboard.putNumber("Right Trigger Value", controller.getRawAxis(controllerMapping.rightTrigger));


		// smartDashboard.("leftEncoder", encoder.getLeftEncoder());
		// smartDashboard.putNumber("rightEncoder",
		// tapeMeasure.rearClimber.get());

		System.out.println("Robot.java");
	}

	/**
	 * This function is called periodically during test mode
	 */
	public void testPeriodic() {
		LiveWindow.run();

	}

}