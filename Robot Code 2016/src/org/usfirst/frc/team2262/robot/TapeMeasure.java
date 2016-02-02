package org.usfirst.frc.team2262.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Talon;

public class TapeMeasure {

	Talon frictionWheel;
	Talon frontClimber;
	Talon rearClimber;

	public TapeMeasure(int frictionWheelChannel, int frontClimberChannel, int rearClimberChannel) {

		frictionWheel = new Talon(frictionWheelChannel);
		frontClimber = new Talon(frontClimberChannel);
		rearClimber = new Talon(rearClimberChannel);

	}

	public void pushUp(boolean pushUpButton) {

		double frictionWheelPushSpeed = 0.25;
		double climberPushSpeed = 0.5;

		if (pushUpButton) {
			frictionWheel.set(frictionWheelPushSpeed);
			frontClimber.set(climberPushSpeed);
			rearClimber.set(climberPushSpeed);
		} else
			frictionWheel.set(0);
		frontClimber.set(0);
		rearClimber.set(0);
	}

	public void pullDown(boolean pullDownButton) {

		double frictionWheelPullSpeed = -0.4;
		double climberPullSpeed = -0.8;

		if (pullDownButton) {
			frictionWheel.set(frictionWheelPullSpeed);
			frontClimber.set(climberPullSpeed);
			rearClimber.set(climberPullSpeed);
		} else
			frictionWheel.set(0);
		frontClimber.set(0);
		rearClimber.set(0);
	}

}
