package org.usfirst.frc.team2262.robot;

import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Joystick;

public class Drive {

	CANTalon frontLeft;
	CANTalon rearLeft;
	CANTalon frontRight;
	CANTalon rearRight;

	RobotDrive drive;

	Joystick joystick;


	public Drive(int frontLeftChannel, int rearLeftChannel, int frontRightChannel, int rearRightChannel,
			int joystickPort) {

		frontLeft = new CANTalon(frontLeftChannel);
		rearLeft = new CANTalon(rearLeftChannel);
		frontRight = new CANTalon(frontRightChannel);
		rearRight = new CANTalon(rearRightChannel);

		drive = new RobotDrive(frontLeft, rearLeft, frontRight, rearRight);

		joystick = new Joystick(joystickPort);
		
		System.out.println("Drive.java");

	}
	
	public void driveForward(double speed) {
		frontLeft.set(speed);
		rearLeft.set(speed);
		frontRight.set(speed);
		rearRight.set(speed);
		
	}
	
	public void stop(){
		frontLeft.set(0);
		rearLeft.set(0);
		frontRight.set(0);
		rearRight.set(0);
	}
	
	public void turnRight(double speed){
		frontLeft.set(speed);
		rearLeft.set(speed);
		frontRight.set((double)-1*speed);
		rearRight.set((double)-1*speed);
		
	}
	
	public void turnLeft(double speed){
		frontLeft.set((double)-1*speed);
		rearLeft.set((double)-1*speed);
		frontRight.set(speed);
		rearRight.set(speed);
	}

	public void driveMotion() {

		drive.arcadeDrive(joystick);

	}
}
