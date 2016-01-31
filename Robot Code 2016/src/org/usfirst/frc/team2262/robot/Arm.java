package org.usfirst.frc.team2262.robot;

import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.DigitalInput;

public class Arm extends Robot{//does it need to extend Robot??; probably yes

	Talon elbow; 
	Talon armRoller;
	
	DigitalInput limitSwitchTop;
	DigitalInput limitSwitchBottom;
	
public Arm(int elbowChannel, int armRollerChannel, int topChannel, int bottomChannel){

    elbow = new Talon(elbowChannel);
    armRoller = new Talon(armRollerChannel);
    
    limitSwitchTop = new DigitalInput(topChannel);
    limitSwitchBottom = new DigitalInput(bottomChannel);     
}

public void elbowMotion(boolean up, boolean down){
	
	double elbowSpeed = 0.1;
	
	if (limitSwitchTop.get() && up) {
		elbow.set(0);
	}else if (up) {
		elbow.set(elbowSpeed);
	}
	
	if (limitSwitchBottom.get() && down) {
		elbow.set(0);
	}else if (down) {
		elbow.set(-elbowSpeed);
	}
}
}
