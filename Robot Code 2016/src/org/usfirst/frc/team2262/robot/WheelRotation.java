package org.usfirst.frc.team2262.robot;

import edu.wpi.first.wpilibj.Encoder;

public class WheelRotation {
	private Encoder leftEncoder;
	private Encoder rightEncoder;
	
	double distancePerPulse;  
	
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
	
	public WheelRotation(double wheelDiameter, double pulsePerRevolution){
		
		leftEncoder = new Encoder(0,1);
		rightEncoder = new Encoder(2,3);
		
		distancePerPulse = Math.PI*wheelDiameter/pulsePerRevolution; 
		
		leftEncoder.setDistancePerPulse(distancePerPulse);
		rightEncoder.setDistancePerPulse(distancePerPulse);
		
		leftEncoder.reset();
		rightEncoder.reset();	
		
		
	}
	
	public double getDistance(){
		double distance = (leftEncoder.getDistance() + rightEncoder.getDistance()) / (double)2;
		return distance;
	}
	
}
