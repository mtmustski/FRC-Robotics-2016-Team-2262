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

		double frictionWheelPushSpeed = 0.4; //0.4
		double climberPushSpeed = 0;
		
		frictionWheel.set(frictionWheelPushSpeed);
		frontClimber.set(climberPushSpeed);
		rearClimber.set(climberPushSpeed);
		
		//for(double i = -0.5; i< -1.0; i -=0.02){
			//frictionWheel.set(i); }
		
		/*double climberPushSpeed = 0;
		
		if (pushUpButton) {
			
			double currentSpeed = frictionWheel.get();
			double desiredSpeed = -1;
			double speedIncrement = -0.02;
			double outputSpeed = 0;
			
			outputSpeed = currentSpeed + speedIncrement;
			
			if (outputSpeed < desiredSpeed) {
				outputSpeed = desiredSpeed;
			}
			
			
			
			frictionWheel.set(outputSpeed);
			frontClimber.set(climberPushSpeed);
			rearClimber.set(climberPushSpeed);
			
			} else {
				
				double currentSpeed = frictionWheel.get();
				double desiredSpeed = 0;
				double speedIncrement = 0.02;
				double outputSpeed = 0;
				
				outputSpeed = currentSpeed + speedIncrement;
				
				if (outputSpeed > desiredSpeed) {
					outputSpeed = desiredSpeed;
				}
				
				frictionWheel.set(outputSpeed);
				frontClimber.set(climberPushSpeed);
				rearClimber.set(climberPushSpeed);
			}*/
	}
			
			
		//}
	

	public void pullDown(boolean pullDownButton) {

		double frictionWheelPullSpeed = 0;
		double climberPullSpeed = -0.5; //-0.5

		/*if (pullDownButton) {
			double currentSpeed = frontClimber.get();
			double desiredSpeed = -1;
			double speedIncrement = -0.02;
			double outputSpeed = 0;
			
			outputSpeed = currentSpeed + speedIncrement;
			
			if (outputSpeed < desiredSpeed) {
				outputSpeed = desiredSpeed;
			}
			
			frictionWheel.set(frictionWheelPullSpeed);
			frontClimber.set(outputSpeed);
			rearClimber.set(outputSpeed);
			
			} else {
				
				double currentSpeed = frontClimber.get();
				double desiredSpeed = 0;
				double speedIncrement = 0.02;
				double outputSpeed = 0;
				
				outputSpeed = currentSpeed + speedIncrement;
				
				if (outputSpeed > desiredSpeed) {
					outputSpeed = desiredSpeed;
				}
				
				frictionWheel.set(frictionWheelPullSpeed);
				frontClimber.set(outputSpeed);
				rearClimber.set(outputSpeed);*/
			
			
			
			//frictionWheel.set(frictionWheelPullSpeed);
			//for(double i = -0.5; i< -1.0; i -=0.02){
			//frontClimber.set(i);
			//rearClimber.set(i);
		
		frictionWheel.set(frictionWheelPullSpeed);
		frontClimber.set(climberPullSpeed);
		rearClimber.set(climberPullSpeed);
		
		//}
	}

}
