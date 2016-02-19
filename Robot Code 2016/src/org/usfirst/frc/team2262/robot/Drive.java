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
	
	WheelRotation encoder;
	
	//drive control variables
	double rawLeftVoltage = 0;
	double rawRightVoltage = 0;
	double processedLeftVoltage = 0;
	double processedRightVoltage = 0;
	double outputLeftVoltage = 0;
	double outputRightVoltage = 0;
	
	//drive control constants
	double maxLeftSpeed = 88; //inches per second
	double maxRightSpeed = 88;
	
	//drive control arrays
	double[] rawVoltage = { 0 , 0 };
	double[] processedVoltage = { 0 , 0 };
	double[] outputVoltage = { 0 , 0 };
	
			
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
		
		encoder = new WheelRotation (6, 360);
		
	}
	
	public void driveForward(double speed) {
		frontLeft.set((double)-1* speed);
		rearLeft.set((double)-1*speed);
		frontRight.set((double)-1*speed);
		rearRight.set((double)-1*speed);
		
	}
	
	public void stop(){
		frontLeft.set(0);
		rearLeft.set(0);
		frontRight.set(0);
		rearRight.set(0);
	}
	
	public void turnRight(double speed){
		frontLeft.set((double)-1*speed);
		rearLeft.set((double)-1*speed);
		frontRight.set(speed);
		rearRight.set(speed);
		
	}
	
	public void turnLeft(double speed){
		frontLeft.set(speed);
		rearLeft.set(speed);
		frontRight.set((double)-1*speed);
		rearRight.set((double)-1*speed);
	}
	
	public void directInputDrive() {
		
		drive.arcadeDrive(joystick, false);
		
	}

	public void squaredInputDrive() {

		drive.arcadeDrive(joystick, true);

	}
	
	public void controlledInputDrive(){
		
		//for all arrays: 0 = left, 1 = right
		processedVoltage = speedControl(rawVoltage[0] * maxLeftSpeed, rawVoltage[1] * maxRightSpeed, encoder.getLeftSpeed(), encoder.getRightSpeed(), rawVoltage[0], rawVoltage[1]);
		outputVoltage = accelerationControl(outputVoltage[0], outputVoltage[1], processedVoltage[0], processedVoltage[1]);
		rawVoltage = arcadeDrive();
		
		frontLeft.set(outputVoltage[0]);
		rearLeft.set(outputVoltage[0]);
		frontRight.set(outputVoltage[1]);
		rearRight.set(outputVoltage[1]);
		
	}
	
	
	private double[] arcadeDrive() {

		double moveValue = joystick.getY();
		double rotateValue = joystick.getX();

		if (moveValue > 0.0) {
			if (rotateValue > 0.0) {
				rawLeftVoltage = moveValue - rotateValue;
				rawRightVoltage = Math.max(moveValue, rotateValue);
			} else {
				rawLeftVoltage = Math.max(moveValue, -rotateValue);
				rawRightVoltage = moveValue + rotateValue;
			}
		} else {
			if (rotateValue > 0.0) {
				rawLeftVoltage = -Math.max(-moveValue, rotateValue);
				rawRightVoltage = moveValue + rotateValue;
			} else {
				rawLeftVoltage = moveValue - rotateValue;
				rawRightVoltage = -Math.max(-moveValue, -rotateValue);
			}
		}
		return new double[] { rawLeftVoltage, rawRightVoltage };

	}

	private double[] speedControl(double desiredLeftSpeed, double desiredRightSpeed, double currentLeftSpeed, double currentRightSpeed, double rawLeftVoltage, double rawRightVoltage) {
		
		double kSpeed = 1.8;
		
		if (desiredLeftSpeed > 0) {
			if (currentLeftSpeed >= 0) {
				if (desiredLeftSpeed > currentLeftSpeed) {
					processedLeftVoltage= kSpeed * rawLeftVoltage;
				}
				if (desiredLeftSpeed < currentLeftSpeed) {
					processedLeftVoltage=0;
				}
			}
			if (currentLeftSpeed < 0) {
				processedLeftVoltage=0;
			}
		}
		
		if (desiredLeftSpeed < 0) {
			if (currentLeftSpeed <= 0) {
				if (desiredLeftSpeed < currentLeftSpeed) {
					processedLeftVoltage= kSpeed * rawLeftVoltage;
				}
				if (desiredLeftSpeed > currentLeftSpeed) {
					processedLeftVoltage=0;
				}
			}
			if (currentLeftSpeed > 0) {
				processedLeftVoltage=0;
			}
		}
		
		if (desiredLeftSpeed > 0) {
			if (currentRightSpeed >= 0) {
				if (desiredRightSpeed > currentRightSpeed) {
					processedRightVoltage= kSpeed * rawRightVoltage;
				}
				if (desiredRightSpeed < currentRightSpeed) {
					processedRightVoltage=0;
				}
			}
			if (currentRightSpeed < 0) {
				processedRightVoltage=0;
			}
		}
		
		if (desiredRightSpeed < 0) {
			if (currentRightSpeed <= 0) {
				if (desiredRightSpeed < currentRightSpeed) {
					processedRightVoltage= kSpeed * rawRightVoltage;
				}
				if (desiredRightSpeed > currentRightSpeed) {
					processedRightVoltage=0;
				}
			}
			if (currentRightSpeed > 0) {
				processedRightVoltage=0;
			}
		}
		
		if (processedLeftVoltage > 1) {
			processedLeftVoltage = 1;
		}
		if (processedLeftVoltage < -1) {
			processedLeftVoltage = -1;
		}
		if (processedRightVoltage > 1) {
			processedRightVoltage = 1;
		}
		if (processedRightVoltage < -1) {
			processedRightVoltage = -1;
		}
		
		return new double[] { processedLeftVoltage, processedRightVoltage };
		
	}
	
	private double[] accelerationControl (double previousLeftVoltage, double previousRightVoltage, double processedLeftVoltage, double processedRightVoltage) {
		
		double kMaxAcceleration = 0.025;  //time delay constant between full back and full forward: .01=4seconds   .015=3seconds   .02=2seconds ??
		
		if (Math.abs(processedLeftVoltage - previousLeftVoltage) > kMaxAcceleration){
			if (processedLeftVoltage > previousLeftVoltage) {
				outputLeftVoltage = previousLeftVoltage + kMaxAcceleration;
				} 
			if (processedLeftVoltage < previousLeftVoltage) {
				outputLeftVoltage = previousLeftVoltage - kMaxAcceleration;
				}
		}	else {
			outputLeftVoltage = processedLeftVoltage;
		
		}
		
		if (Math.abs(processedRightVoltage - previousRightVoltage) > kMaxAcceleration){
			if (processedRightVoltage > previousRightVoltage) {
				outputRightVoltage = previousRightVoltage + kMaxAcceleration;
				} 
			if (processedRightVoltage < previousRightVoltage) {
				outputRightVoltage = previousRightVoltage - kMaxAcceleration;
				}
		}	else {
			outputRightVoltage = processedRightVoltage;
		}
		return new double[] { outputLeftVoltage, outputRightVoltage };
	
	}

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
