package org.usfirst.frc.team2262.robot;

import edu.wpi.first.wpilibj.Encoder;

public class WheelRotation {
	private Encoder leftEncoder;
	private Encoder rightEncoder;
	
	double inchesPerPulse;  
	
	public Encoder getLeftEncoder() {
		return leftEncoder;
	}
	public void setLeftEncoder(Encoder leftEncoder) {
		this.leftEncoder = leftEncoder;
	}
	public Encoder getRightEncoder() {
		return rightEncoder;
	}
	public void setRightEncoder(Encoder rightEncoder) {
		this.rightEncoder = rightEncoder;
	}
	
	public WheelRotation(double wheelDiameter, double pulsesPerRevolution){
		
		leftEncoder = new Encoder(0, 1, false);
		rightEncoder = new Encoder(2, 3, false);
		
		inchesPerPulse = Math.PI*wheelDiameter/pulsesPerRevolution; 
		
		leftEncoder.setDistancePerPulse(inchesPerPulse);
		rightEncoder.setDistancePerPulse(inchesPerPulse);
		
		leftEncoder.reset();
		rightEncoder.reset();	
		
		
	}
	
	public double getDistance() {
		double distance = (leftEncoder.getDistance() + rightEncoder.getDistance()) / (double)2;
		return distance;
	}
	
	public double getLeftSpeed() {
		double leftSpeed = leftEncoder.getRate(); //inches per second
		return leftSpeed;
	}
	
	public double getRightSpeed() {
		double rightSpeed = rightEncoder.getRate(); //inches per second
		return rightSpeed;
	}
	
}
