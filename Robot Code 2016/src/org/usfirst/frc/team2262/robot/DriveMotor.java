package org.usfirst.frc.team2262.robot;

import edu.wpi.first.wpilibj.TalonSRX;

public class DriveMotor {
	private TalonSRX front;
	private TalonSRX back;

	public DriveMotor(int frontChannel, int backChannel) {
		front = new TalonSRX(frontChannel);
		back = new TalonSRX(backChannel);

	}

	public void stop() {
		front.stopMotor();
		back.stopMotor();
	}

}
