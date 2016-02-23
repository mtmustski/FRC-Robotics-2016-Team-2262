package org.usfirst.frc.team2262.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TapeMeasure {

	Talon frictionWheel;
	Talon frontClimber;
	Talon rearClimber;

	public TapeMeasure(int frictionWheelChannel, int frontClimberChannel, int rearClimberChannel) {

		frictionWheel = new Talon(frictionWheelChannel);
		frontClimber = new Talon(frontClimberChannel);
		rearClimber = new Talon(rearClimberChannel);
		
		frictionWheel.setInverted(true);
		frontClimber.setInverted(true);
		rearClimber.setInverted(true);
		
	}

	public void pushUp(boolean pushUpButton) {

		double frictionWheelPushSpeed = 0.4;
		double climberPushSpeed = 0;

		if (pushUpButton) {
			frictionWheel.set(frictionWheelPushSpeed);
			frontClimber.set(climberPushSpeed);
			rearClimber.set(climberPushSpeed);
		
		}
	}

	public void pullDown(boolean pullDownButton) {

		double frictionWheelPullSpeed = 0;
		double climberPullSpeed = -1;

		if (pullDownButton) {
			frictionWheel.set(frictionWheelPullSpeed);
			frontClimber.set(climberPullSpeed);
			rearClimber.set(climberPullSpeed);
		}
	}

}
