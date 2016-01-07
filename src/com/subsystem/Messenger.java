package com.subsystem;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Messenger extends Subsystem {
	
	private double targetHeightVolt;
	private double heading;
	private double targetAngle;
	private boolean atLocation;
	private boolean isOrienting;
	private double potVoltage;
	private String elevatorMode;
    public Messenger() {
        DriverStation.getInstance();
    }
    
    public void putData() {
    	if (stick2.getRawButton(1)){
    		elevatorMode = "Auto";
    	} else if (stick2.getRawButton(9)){
    		elevatorMode = "Manual";
    	} else {
    		elevatorMode = "Off";
    	}
    	try{
    		
    		//SmartDashboard.putNumber("Joystick Will Motor Set Voltage", 2 * Math.log(Math.abs(linearTransform(stick2.getRawAxis(0), -1, 0.866, 4.419, 1.344) - potVoltage)));
    		//SmartDashboard.putNumber("Joystick Target Pot Angle", linearTransform(stick2.getRawAxis(0), -1, 0.866, 1.444, 0.543));
    		SmartDashboard.putNumber("Actual Target Pot Height", targetHeightVolt);
    		SmartDashboard.putNumber("Slide Value", stick2.getRawAxis(0));
	    	SmartDashboard.putNumber("Heading", heading);
	    	SmartDashboard.putNumber("Target Angle", targetAngle);
	    	SmartDashboard.putBoolean("At location", atLocation);
	    	SmartDashboard.putBoolean("Is orienting", isOrienting);
	    	SmartDashboard.putNumber("Pot voltage", potVoltage);
	    	SmartDashboard.putString("Elevator Control Mode", elevatorMode);
    	} catch (Exception ex){
    	}
    }
    public void setData(double heading, boolean atLocation, double targetAngle, boolean isOrienting, double potVoltage, double targetHeightVolt){
    	this.targetHeightVolt = targetHeightVolt;
    	this.heading = heading;
    	this.atLocation = atLocation;
    	this.targetAngle = targetAngle;
    	this.isOrienting = isOrienting;
    	this.potVoltage = potVoltage;
    }
    

}