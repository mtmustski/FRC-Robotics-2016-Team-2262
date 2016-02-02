package org.usfirst.frc.team2262.robot;

import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.DigitalInput;

public class Arm {

	// Iterative extends Arm
	Talon elbow;
	Talon armRoller;

	DigitalInput limitSwitchTop;
	DigitalInput limitSwitchBottom;

	public Arm(int elbowChannel, int armRollerChannel, int topChannel, int bottomChannel) {

		elbow = new Talon(elbowChannel);
		armRoller = new Talon(armRollerChannel);

		limitSwitchTop = new DigitalInput(topChannel);
		limitSwitchBottom = new DigitalInput(bottomChannel);
	}

	public void elbowMotion(double triggerValue) {

		double maxElbowSpeed = 0.3;

		if (limitSwitchBottom.get() && triggerValue > 0) {
			elbow.set(0);
		} else if (triggerValue > 0) {
			elbow.set(-triggerValue * maxElbowSpeed);
		}

		if (limitSwitchTop.get() && triggerValue < 0) {
			elbow.set(0);
		} else if (triggerValue < 0) {
			elbow.set(-triggerValue * maxElbowSpeed);
		}		
	}

}
