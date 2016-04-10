package org.usfirst.frc.team2262.robot;

import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.DigitalInput;

public class Arm {

	Talon elbow;
	Talon roller;

	DigitalInput limitSwitchTop;
	//DigitalInput limitSwitchBottom;

	public Arm(int elbowChannel, int armRollerChannel, int topChannel) {

		elbow = new Talon(elbowChannel);
		roller = new Talon(armRollerChannel);

		limitSwitchTop = new DigitalInput(topChannel);
		//limitSwitchBottom = new DigitalInput(bottomChannel);
	}

	public void ballIntake(double intakeTrigger) {

		double elbowIntakeSpeed = -0.135;
		double rollerIntakeSpeed = -0.75;

		if (intakeTrigger > 0) {
			elbow.set(elbowIntakeSpeed);
			roller.set(rollerIntakeSpeed);
		}
	}

	public void ballOutput(boolean outputButton) {

		double elbowOutputSpeed = 0.1;
		double rollerOutputSpeed = 0.9;

		if (!limitSwitchTop.get() && outputButton) {
			elbow.set(0);
			roller.set(rollerOutputSpeed);
		} else if (outputButton) {
			elbow.set(elbowOutputSpeed);
			roller.set(rollerOutputSpeed);
		}
	}

	public void elbowMotion(double triggerDown, boolean buttonUp) {

		double maxElbowSpeed = 0.25;

		// elbow down
		if (triggerDown > 0) {
				elbow.set(-maxElbowSpeed);
		}

		// elbow up
		if (buttonUp) {
			if (!limitSwitchTop.get()) {
				elbow.set(0);
			} else {
				elbow.set(maxElbowSpeed);
			}
		}
	}

	public void rollerMotion(boolean rollerInButton, boolean rollerOutButton) {

		double maxRollerSpeed = 0.75;

		if (rollerInButton) {
			roller.set(-maxRollerSpeed);
		}

		if (rollerOutButton) {
			roller.set(maxRollerSpeed);
		}
	}
	
	public void powerArm (boolean powerArmButton) {

		double maxElbowSpeed = -1;

		if (powerArmButton) {
			elbow.set(-maxElbowSpeed);
		}

	}
	
	public void stopArmMotion () {
		
		elbow.set(0);
		roller.set(0);
		
	}

}
