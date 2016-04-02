package org.usfirst.frc.team2262.robot;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;

//import adis16448.frc.ADIS16448_IMU;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
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

	// ip address of pie
	// 10.22.62.15

	NetworkTable table;
	double centerXValue;

	public Robot() {
		table = NetworkTable.getTable("GRIP/myContoursReport");
	}

	DriverStation.Alliance allianceColor;

	Drive drive;

	CameraServer server;
	
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
	double angleFudgeFactor = 0;
	double driveForwardSpeed = 0;
	double driveCorrectionSpeed = 0;
	double driveTurnSpeed = 0;

	enum AutonomousState {
		LowerArm, MoveOverLowBar, TurnToTower, MoveToTower, Aim, Shoot, Done, MoveToX
	};

	AutonomousState myState = AutonomousState.LowerArm;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	public void robotInit() {

		allianceColor = DriverStation.getInstance().getAlliance();

		drive = new Drive(0, 1, 2, 3, 0);

		imu = new ADIS16448_IMU();

		imu.calibrate();

		
		
		arm = new Arm(0, 1, 4);

		controller = new Joystick(1);

		controllerMapping = new ControllerMapping();

		tapeMeasure = new TapeMeasure(2, 3, 4);

		encoder = new WheelRotation(6, 2035);

		// declaring encoder distancePerPulse
		// double distancePerPulse;
		server = CameraServer.getInstance();
		server.setQuality(50);
		server.startAutomaticCapture("cam0");
	}

	/**
	 * This function is run once each time the robot enters autonomous mode
	 */
	public void autonomousInit() {
		// imu.calibrate();
		myState = AutonomousState.LowerArm;
		autoLoopCounter = 0;
		distance = 20;
		howFarToGo = 0;
		armLowerTimer = 0;

		elapsedTime = 0;
		timerStart = Timer.getFPGATimestamp();

		encoder.reset();
		// displaying the original angle of the imu on the dash

		imuAngleOriginal = imu.getAngleZ();
		SmartDashboard.putNumber("Imu Angle Original", imuAngleOriginal);

		// angle we need to turn to for tower
		desiredDegrees = 60;

		angleFudgeFactor = 7;

		driveForwardSpeed = .5;
		driveCorrectionSpeed = 0.1;
		driveTurnSpeed = .2;
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
		elapsedTime = Timer.getFPGATimestamp() - timerStart;
	
		/*
		//low bar mode
		if (elapsedTime < 1) {
			arm.elbowMotion(1, false);
		} else {
			arm.stopArmMotion();
		}

		if(elapsedTime > 1 && elapsedTime < 6){
			drive.driveForward(0.35);
		} else {
			drive.stop();
		}*/
		
		//reach mode
		/*if(elapsedTime > 1 && elapsedTime < 4){
			drive.driveForward(0.3);
		} else {
			drive.stop();
		}*/
		
		//see-saw mode
		/*if(elapsedTime > 1 && elapsedTime < 3.5){
			drive.driveForward(0.3);
		} else {
			drive.stop();
		}
		
		if (elapsedTime > 3.5 && elapsedTime < 4.5) {
			arm.elbowMotion(1, false);
		} else {
			arm.stopArmMotion();
		}
		
		if(elapsedTime > 4.5 && elapsedTime < 7){
			drive.driveForward(0.4);
		} else {
			drive.stop();
		}*/
		
		//gate mode
		/*if (elapsedTime < 1) {
			arm.elbowMotion(1, false);
		} else {
			arm.stopArmMotion();
		}

		if(elapsedTime > 1 && elapsedTime < 4.5){
			drive.driveForward(0.7);
		} else {
			drive.stop();
		}*/
		
		double[] defaultValue = new double[0];
		centerXValue = 100.0;
		boolean  foundTarget  = false;
		double[] centerX = table.getNumberArray("centerX", defaultValue);
		for (double d : centerX) {
			centerXValue = d;
			foundTarget = true;
		}

		imuAngleCurrent = imu.getAngleZ();
		SmartDashboard.putNumber("Imu Angle Current", imuAngleCurrent);
		double imuAngleDiffrence = imuAngleCurrent - imuAngleOriginal;
		SmartDashboard.putNumber("Imu Angle Diffrence", imuAngleDiffrence);
		elapsedTime = Timer.getFPGATimestamp() - timerStart;
		SmartDashboard.putNumber("Average Encoder Distance", encoder.getDistance());

		switch (myState) {
		case LowerArm:
			SmartDashboard.putString("Auto Case", "Lower Arm");
			if (elapsedTime < 1) {
				arm.elbowMotion(1, false);
			} else {
				arm.stopArmMotion();
				myState = AutonomousState.MoveOverLowBar;
			}
			break;
		case MoveOverLowBar:

			SmartDashboard.putString("Auto Case", "Move Over Low Bar");
			if (elapsedTime > 6) { //(encoder.getDistance() > 88) { // 19.5" is distance from start
												// to over lowgaol? ask trevor
				drive.stop();
				encoder.reset();
				myState = AutonomousState.MoveToX;

			} else {
				/*if (imuAngleDiffrence > angleFudgeFactor) {  //angleFudgeFactor is set high (7 degrees)

					drive.smallTurnLeft(driveForwardSpeed, driveTurnSpeed);
				} else if (imuAngleDiffrence < angleFudgeFactor) {
					drive.smallTurnRight(driveForwardSpeed, driveTurnSpeed);
				} else
					drive.driveForward(driveForwardSpeed);*/
				drive.driveForward(0.4);
			}

			// Have I turned?
			// How far have I gone?
			// How far do I have to go?
			// have I got there?
			// If yes. myState = TurnToTower;

			break;
		case MoveToX:
			SmartDashboard.putString("Auto Case", "Move To X");
			if (elapsedTime > 6.5){ //(encoder.getDistance() > 173) { // 220.75 distance from after
													// lowgaol to turn area ask
													// trevor
				drive.stop();

				myState = AutonomousState.TurnToTower;

			} else {
				/*if (imuAngleDiffrence > angleFudgeFactor) {  //angleFudgeFactor is set high (7 degrees)
					drive.smallTurnLeft(driveForwardSpeed, driveTurnSpeed);
				} else if (imuAngleDiffrence < angleFudgeFactor) {
					drive.smallTurnRight(driveForwardSpeed, driveTurnSpeed);
				} else
					drive.driveForward(driveForwardSpeed);*/
				drive.stop();
			}
			break;
		case TurnToTower:
			SmartDashboard.putString("Auto Case", "Turn To Tower");
			if (Math.abs(imuAngleDiffrence - desiredDegrees) < 1) {
				drive.stop();
				encoder.reset();
				myState = AutonomousState.MoveToTower;
			} else {
				if (imuAngleDiffrence < desiredDegrees)
					drive.turnRight(driveTurnSpeed);

				else if (imuAngleDiffrence > desiredDegrees) {
					drive.turnLeft(driveTurnSpeed);
				}
			}
			// Have I turned enough?
			// If yes. myState = MoveToTower

			break;
		case MoveToTower:
			SmartDashboard.putString("Auto Case", "Move To Tower");

			if (encoder.getDistance() > 35) { // figure out this 10 number
				drive.stop();
				encoder.reset();
				myState = AutonomousState.Aim;

			} else {
				if (imuAngleDiffrence > angleFudgeFactor) {  //angleFudgeFactor is set high (7 degrees)
					drive.smallTurnLeft(driveForwardSpeed, driveTurnSpeed);
				} else if (imuAngleDiffrence < angleFudgeFactor) {
					drive.smallTurnRight(driveForwardSpeed, driveTurnSpeed);
				} else
					drive.driveForward(driveForwardSpeed);
			}
			break;

		// Have I turned?
		// How far have I gone?
		// How far do I have to go?
		// Have I got there.
		// If yes. myState = Aim

		case Aim:
			SmartDashboard.putString("Auto Case", "Aim. FT: " + foundTarget 
					+ " CXV " + centerXValue);
			
			if(foundTarget == false){
				drive.driveForward(driveForwardSpeed);
			}
			else if(centerXValue == 81){
				drive.driveForward(driveForwardSpeed);
			}
			else if(centerXValue > 81.0){  //might oscillate too much
				if (centerXValue < 90)
					drive.turnLeft(driveTurnSpeed / 3);
				else if (centerXValue < 110)
					drive.turnLeft(driveTurnSpeed / 2);
				else
					drive.turnLeft(driveTurnSpeed);
			} else {
				if (centerXValue > 70)
					drive.turnRight(driveTurnSpeed / 3);
				else if (centerXValue > 50)
					drive.turnRight(driveTurnSpeed / 2);
				else
					drive.turnRight(driveTurnSpeed);
			}
			if(encoder.getDistance() > 85){ //distance from aim to shoot, ask  trevor
				drive.stop();
				myState = AutonomousState.Shoot;
			}

			// Aim correct?
			// If yes. myState = shoot;
			// If no.
			// Need to trun left? if so turn
			// Need to Turn right? if so turn
			timerStart = Timer.getFPGATimestamp();
			break;
		case Shoot:
			SmartDashboard.putString("Auto Case", "Shoot");

			// Shoot;

			if (elapsedTime < .3) {  //a lot of options here: 1) no ballOutput, just rollers 2) running into wall to have solid contact 3) backing up from the wall to not get stuck
				arm.ballOutput(true);
			}
			
			myState = AutonomousState.Done;
			break;
		case Done:
			SmartDashboard.putString("Auto Case", "Done");
			// flash lights.
			// Sound horn.
		}

	}

	/**
	 * This function is called once each time the robot enters tele-operated
	 * mode
	 */
	public void teleopInit() {

		tapeMeasure.frictionWheel.set(0);
		tapeMeasure.frontClimber.set(0);
		tapeMeasure.rearClimber.set(0);
		/*
		 * leftDrive.stop(); rightDrive.stop();
		 */

	}

	/**
	 * This function is called periodically during operator control
	 */
	public void teleopPeriodic() {

		imuAngleCurrent = imu.getAngleZ();

		SmartDashboard.putNumber("Imu Angle Current", imuAngleCurrent);
		double imuAngleDiffrence = imuAngleCurrent - imuAngleOriginal;
		SmartDashboard.putNumber("Imu Angle Diffrence", imuAngleDiffrence);
		elapsedTime = Timer.getFPGATimestamp() - timerStart;
		SmartDashboard.putNumber("Average Encoder Distance", encoder.getDistance());

		
		drive.controlledArcadeDrive(controller.getRawAxis(controllerMapping.leftY), controller.getRawAxis(controllerMapping.rightX));
		
		drive.brakeMode(controller.getRawButton(controllerMapping.buttonStart));

		// go forward with button

		// arm code
		boolean leftBumper = controller.getRawButton(controllerMapping.leftBumper);
		boolean rightBumper = controller.getRawButton(controllerMapping.rightBumper);
		double leftTrigger = controller.getRawAxis(controllerMapping.leftTrigger);
		double rightTrigger = controller.getRawAxis(controllerMapping.rightTrigger);
		boolean buttonX = controller.getRawButton(controllerMapping.buttonX);
		boolean buttonB = controller.getRawButton(controllerMapping.buttonB);

		if (rightTrigger > 0) {
			arm.ballIntake(rightTrigger);
		}

		if (rightBumper == true) {
			arm.ballOutput(rightBumper);
		}

		if (leftTrigger != 0.0 || leftBumper == true) {
			arm.elbowMotion(leftTrigger, leftBumper);
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

		//tapeMeasure.pushUp(controller.getRawButton(controllerMapping.buttonY));
		//tapeMeasure.pullDown(controller.getRawButton(controllerMapping.buttonA));
		
		if (!(buttonY || buttonA)) {
			tapeMeasure.frictionWheel.set(0);
			tapeMeasure.frontClimber.set(0);
			tapeMeasure.rearClimber.set(0);
		}

		imu.getAngle();


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

		SmartDashboard.putNumber("Left Encoder Rate", encoder.getLeftEncoder().getRate());
		SmartDashboard.putNumber("Right Encoder Rate", encoder.getRightEncoder().getRate());

		SmartDashboard.putNumber("Left Encoder Distance", encoder.getLeftEncoder().getDistance());
		SmartDashboard.putNumber("Right Encoder Distance", encoder.getRightEncoder().getDistance());
		
		SmartDashboard.putNumber("Joystick X Axis", drive.joystick.getX());
		SmartDashboard.putNumber("Joystick Y Axis", drive.joystick.getY());

		if (allianceColor == DriverStation.Alliance.Blue) {
			SmartDashboard.putString("Alliance Color", "Blue");
		}

		if (allianceColor == DriverStation.Alliance.Red) {
			SmartDashboard.putString("Alliance Color", "Red");
		}

		// smartDashboard.("leftEncoder", encoder.getLeftEncoder());
		// smartDashboard.putNumber("rightEncoder",
		// tapeMeasure.rearClimber.get());

	}

	/**
	 * This function is called periodically during test mode
	 */
	public void testPeriodic() {
		LiveWindow.run();
		encoder.reset(); 
		imu.calibrate();
		
		
		
		SmartDashboard.putNumber("Joystick X Axis", drive.joystick.getX());
		SmartDashboard.putNumber("Joystick Y Axis", drive.joystick.getY());

		// drive.turnRight(-1*.2);

		// drive.turnRight(.2);

		/*
		 * drive.frontLeft.set(.2); drive.rearLeft.set(.2);
		 * drive.frontRight.set(-1 * .2); drive.rearRight.set(-1 * .2);
		 * SmartDashboard.putNumber("Front Left CAN", drive.frontLeft.get());
		 * SmartDashboard.putNumber("Rear Left CAN", drive.rearLeft.get());
		 * SmartDashboard.putNumber("Front Right CAN", drive.frontRight.get());
		 * SmartDashboard.putNumber("Rear Right CAN", drive.rearRight.get());
		 */
	}

}