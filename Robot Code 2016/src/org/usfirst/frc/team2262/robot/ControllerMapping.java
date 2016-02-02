package org.usfirst.frc.team2262.robot;

public class ControllerMapping {

	// axes (all output is between -1 and 1)
	public int leftX = 1; // left: - ; right: +
	public int leftY = 2; // up: - ; down: +
	public int triggers = 3; // left: + ; right: -
	public int rightX = 4; // left: - ; right: +
	public int rightY = 5; // up: - ; down: +
	public int dPad = 6; // left-right; not recommended, buggy

	// buttons
	public int buttonA = 1;
	public int buttonB = 2;
	public int buttonX = 3;
	public int buttonY = 4;
	public int leftBumper = 5;
	public int rightBumper = 6;
	public int buttonBack = 7;
	public int buttonStart = 8;
	public int buttonLeftJoystick = 9;
	public int buttonRightJoystick = 10;
}
