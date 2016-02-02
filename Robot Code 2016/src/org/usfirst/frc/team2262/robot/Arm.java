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

		if (limitSwitchBottom.get() && intakeButton) {
			elbow.set(0);
			roller.set(rollerIntakeSpeed);
		} else if (intakeButton) {
			elbow.set(elbowIntakeSpeed);
			roller.set(rollerIntakeSpeed);
		} else
			elbow.set(0);
		roller.set(0);
	}

	public void ballOutput(boolean outputButton) {

		double elbowOutputSpeed = 0.2;
		double rollerOutputSpeed = 0.9;

		if (limitSwitchTop.get() && outputButton) {
			elbow.set(0);
			roller.set(rollerOutputSpeed);
		} else if (outputButton) {
			elbow.set(elbowOutputSpeed);
			roller.set(rollerOutputSpeed);
		} else
			elbow.set(0);
		roller.set(0);
	}

	public void elbowMotion(double elbowTrigger) {

		double maxElbowSpeed = 0.3;

		// elbow down
		if (limitSwitchBottom.get() && elbowTrigger > 0) {
			elbow.set(0);
		} else if (elbowTrigger > 0) {
			elbow.set(-elbowTrigger * maxElbowSpeed);
		} else
			elbow.set(0);

		// elbow up
		if (limitSwitchTop.get() && elbowTrigger < 0) {
			elbow.set(0);
		} else if (elbowTrigger < 0) {
			elbow.set(-elbowTrigger * maxElbowSpeed);
		} else
			elbow.set(0);
	}

	public void rollerMotion(boolean rollerInButton, boolean rollerOutButton) {

		double maxRollerSpeed = 0.9;

		if (rollerInButton) {
			roller.set(-maxRollerSpeed);
		} else
			roller.set(0);

		if (rollerOutButton) {
			roller.set(maxRollerSpeed);
		} else
			roller.set(0);
	}

}
