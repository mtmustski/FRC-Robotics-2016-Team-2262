package org.usfirst.frc.team2262.robot;

import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.DigitalInput;

public class Arm {

	Talon elbow;
	Talon roller;

	DigitalInput limitSwitchTop;
	DigitalInput limitSwitchBottom;

	public Arm(int elbowChannel, int armRollerChannel, int topChannel, int bottomChannel) {

		elbow = new Talon(elbowChannel);
		roller = new Talon(armRollerChannel);

		limitSwitchTop = new DigitalInput(topChannel);
		limitSwitchBottom = new DigitalInput(bottomChannel);
	}

	public void ballIntake(boolean intakeButton) {

		double elbowIntakeSpeed = -0.15;
		double rollerIntakeSpeed = -0.8;

		if (!limitSwitchBottom.get() && intakeButton) {
			elbow.set(0);
			roller.set(rollerIntakeSpeed);
		} else if (intakeButton) {
			elbow.set(elbowIntakeSpeed);
			roller.set(rollerIntakeSpeed);
		}
	}

	public void ballOutput(boolean outputButton) {

		double elbowOutputSpeed = 0.2;
		double rollerOutputSpeed = 0.9;

		if (!limitSwitchTop.get() && outputButton) {
			elbow.set(0);
			roller.set(rollerOutputSpeed);
		} else if (outputButton) {
			elbow.set(elbowOutputSpeed);
			roller.set(rollerOutputSpeed);
		}
	}

	public void elbowMotion(double triggerDown, double triggerUp) {

		double maxElbowSpeed = 0.3;

		// elbow down
		if (triggerDown > 0) {
			if (!limitSwitchBottom.get()) {
				elbow.set(0);
			} else {
				elbow.set(-triggerDown * maxElbowSpeed);
			}
		}

		// elbow up
		if (triggerUp > 0) {
			if (!limitSwitchTop.get()) {
				elbow.set(0);
			} else {
				elbow.set(triggerUp * maxElbowSpeed);
			}
		}
	}

	public void rollerMotion(boolean rollerInButton, boolean rollerOutButton) {

		double maxRollerSpeed = 0.9;

		if (rollerInButton) {
			roller.set(-maxRollerSpeed);
		}

		if (rollerOutButton) {
			roller.set(maxRollerSpeed);
		}
	}

}
