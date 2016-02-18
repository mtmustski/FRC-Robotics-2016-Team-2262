package org.usfirst.frc.team2262.robot;

import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Joystick;

public class Drive {

	CANTalon frontLeft;
	CANTalon rearLeft;
	CANTalon frontRight;
	CANTalon rearRight;

	RobotDrive drive;

	Joystick joystick;


	public Drive(int frontLeftChannel, int rearLeftChannel, int frontRightChannel, int rearRightChannel,
			int joystickPort) {

		frontLeft = new CANTalon(frontLeftChannel);
		rearLeft = new CANTalon(rearLeftChannel);
		frontRight = new CANTalon(frontRightChannel);
		rearRight = new CANTalon(rearRightChannel);
		
		frontLeft.setInverted(true);
		rearLeft.setInverted(true);
		frontRight.setInverted(true);
		rearRight.setInverted(true);
		

		drive = new RobotDrive(frontLeft, rearLeft, frontRight, rearRight);

		joystick = new Joystick(joystickPort);
		
		System.out.println("Drive.java");

	}
	
	public void driveForward(double speed) {
		frontLeft.set(speed);
		rearLeft.set(speed);
		frontRight.set(speed);
		rearRight.set(speed);
		
	}
	
	public void stop(){
		frontLeft.set(0);
		rearLeft.set(0);
		frontRight.set(0);
		rearRight.set(0);
	}
	
	public void turnRight(double speed){
		frontLeft.set(speed);
		rearLeft.set(speed);
		frontRight.set((double)-1*speed);
		rearRight.set((double)-1*speed);
		
	}
	
	public void turnLeft(double speed){
		frontLeft.set((double)-1*speed);
		rearLeft.set((double)-1*speed);
		frontRight.set(speed);
		rearRight.set(speed);
	}

	public void driveMotion() {

		//rive.arcadeDrive(joystick);
		drive.arcadeDrive(joystick, true);
		
		
		/*

		double kVoltage=1.6;	//inc kVoltage: dec sensitivity & inc direction accuracy
								//dec kVoltage: inc sensitivity & dec direction accuracy
				!!! ACCELERATION MULTIPLIER

		double encoderController(double desiredSpeed, double currentSpeed, double rawVoltage){
		double voltage=0;
		if(desiredSpeed>0.2){	!!! DESIRED SPEED IS THE MINIMUM STIMULUS AT WHICH THE ROBOT WOULD MOVE
			if (desiredSpeed>currentSpeed){voltage=kVoltage*rawVoltage;}
			if (desiredSpeed<currentSpeed){voltage=0;}
		}
		if(desiredSpeed<-0.2){
			if (desiredSpeed<currentSpeed){voltage=kVoltage*rawVoltage;}
			if (desiredSpeed>currentSpeed){voltage=0;}
		}
		if (voltage>1) {voltage=1;}
		if (voltage<-1) {voltage=-1;}
		return voltage;
		
		
		double maxRateOfChange=.01;		//time delay constant between full back and full forward: .01=4seconds   .015=3seconds   .02=2seconds
		
		double lowPassFilter(double lastOutputToEsc, double currentOutputToEsc){
		if (Math.abs(currentOutputToEsc-lastOutputToEsc)>maxRateOfChange){
			if (currentOutputToEsc>lastOutputToEsc){currentOutputToEsc=lastOutputToEsc+maxRateOfChange;} 
			else if (currentOutputToEsc<lastOutputToEsc){currentOutputToEsc=lastOutputToEsc-maxRateOfChange;}
		}
		return currentOutputToEsc;
		!!! LIMITS THE MAXIMUM ACCELERATION


		outputToEscFL=lowPassFilter(outputToEscFL, encoderController(frontLeft*driveTopSpeed, frontLeftSpeed, frontLeft));
    	outputToEscFR=lowPassFilter(outputToEscFR, encoderController(-frontRight*driveTopSpeed, frontRightSpeed, -frontRight));
    	outputToEscRL=lowPassFilter(outputToEscRL, encoderController(rearLeft*driveTopSpeed, rearLeftSpeed, rearLeft));
    	outputToEscRR=lowPassFilter(outputToEscRR, encoderController(-rearRight*driveTopSpeed, rearRightSpeed, -rearRight));
    	 	
    	*/

	}
}
