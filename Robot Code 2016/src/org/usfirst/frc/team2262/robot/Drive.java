package org.usfirst.frc.team2262.robot;

import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.TalonSRX;
import edu.wpi.first.wpilibj.Joystick;

public class Drive {

	TalonSRX frontLeft;
	TalonSRX rearLeft;
	TalonSRX frontRight;
	TalonSRX rearRight;

	RobotDrive drive;

	Joystick joystick;

	public Drive(int frontLeftChannel, int rearLeftChannel, int frontRightChannel, int rearRightChannel,
			int joystickPort) {

		frontLeft = new TalonSRX(frontLeftChannel);
		rearLeft = new TalonSRX(rearLeftChannel);
		frontRight = new TalonSRX(frontRightChannel);
		rearRight = new TalonSRX(rearRightChannel);

		drive = new RobotDrive(frontLeft, rearLeft, frontRight, rearRight);

		joystick = new Joystick(joystickPort);

	}

	public void driveMotion() {

		drive.arcadeDrive(joystick);

	}
}
