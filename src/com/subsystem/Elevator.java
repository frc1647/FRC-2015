package com.subsystem;

import edu.wpi.first.wpilibj.*;

public class Elevator extends Subsystem {
	private Solenoid armLeft;
	private Solenoid armRight;
	private Talon liftMotor1;
	private Talon liftMotor2;
	private AnalogInput pot;
	private final double POT_CONVERTION_VALUE = 0.481;
	Timer strafeTimer = new Timer();
	private boolean buttons[];
	private double targetVolt;
	
	public Elevator() {
		armLeft = new Solenoid(0);
		armRight = new Solenoid(1);
		liftMotor1 = new Talon(4);
		liftMotor2 = new Talon(5);
		pot = new AnalogInput(1);
		buttons = new boolean[2];
	}
	
	public void setClaw() { //opened closed. 
		if(stick3.getRawAxis(3) > 0.7) {  //fail safe
			buttons[0] = toggleSolenoid(armLeft, stick3.getRawButton(7), buttons[0]);
			buttons[1] = toggleSolenoid(armRight, stick3.getRawButton(8), buttons[1]);
		} else {
			armLeft.set(!stick2.getRawButton(4));
			armRight.set(!stick2.getRawButton(5));
		}
	}
	
	public void setClaw(boolean armLeftState, boolean armRightState) { //for auto
		armLeft.set(armLeftState);
		armRight.set(armRightState);
	}
	
	public boolean manageStrafe(){
		final double STRAFE_TIME = 0.3;
		if(!armLeft.get()){
			if (strafeTimer.get() == 0.0){
				strafeTimer.start();
			}
		} else {
			strafeTimer.stop();
			strafeTimer.reset();
		}
		return strafeTimer.get() < STRAFE_TIME && strafeTimer.get() != 0.0;
	}
	
	public boolean armHandler() { //called in teleop
		if(stick3.getRawAxis(3) > 0.7) { // fail safe
			if(stick3.getRawButton(1)) {
				liftMotor1.set(stick3.getRawAxis(1));
				liftMotor2.set(liftMotor1.get());
			} else {
				liftMotor1.set(0);
				liftMotor2.set(liftMotor1.get());
			}
			return false;
		}
		if (stick2.getRawButton(1)){
			targetVolt = linearTransform(stick2.getRawAxis(0), -1, 0.866, 4.419, 1.4); // move to joystick position if a switch is turned on.
			liftMotor2.set(liftMotor1.get());
			return motorPControl(targetVolt, liftMotor1, pot, true, false);
		} else if (stick2.getRawButton(9)){
			liftMotor1.set(stick2.getRawAxis(0));
			liftMotor2.set(liftMotor1.get());
			return false;
		} else {
			liftMotor1.set(0.0);
			liftMotor2.set(liftMotor1.get());
			return false;
		}
		
	}
	
	public boolean armHandler(double location, boolean largeThresh) { //for auto
		System.out.println("Handeling");
		boolean value =  motorPControl(location, liftMotor1, pot, true, largeThresh);
		liftMotor2.set(liftMotor1.get());
		return value;
	}
	
    private boolean motorPControl(double targVolt, Talon motor, AnalogInput pot, boolean neg, boolean largeThresh) { //neg if pot and motor are inversely proportional
        final double threashold = largeThresh ? 0.2 : 0.01; //Threshold for zero
        double diff = targVolt - pot.getVoltage();
        if (targVolt == -1){
        	motor.set(0.0);
        	return true;
        }
        else if (diff >= threashold) {
            if (diff > 0.35) {
                motor.set(-0.6);
            } else {
                motor.set((2.0 * Math.log(Math.abs(diff) + 1)) * (neg ? -1 : 1));  //0.1014 * Math.log(Math.abs(diff)) + 0.6343
                System.out.println(motor.get());
            }
            return false;
        } else if (diff <= threashold * -1) { //up
            if (diff < -0.35 ) {
                motor.set(0.8);
            } else {
                motor.set(-(2.0 * Math.log(Math.abs(diff)+1)) * (neg ? -1 : 1));
            }
            return false;
        } else {
            motor.set(0.0);
            return true;
        }
    }
    public double getPot() {
    	return pot.getVoltage();
    }
    public double getTargetVoltage(){
    	return targetVolt;
    }
    
    private boolean toggleSolenoid(Solenoid piston, boolean buttonPressed, boolean buttonHeld) {
        if (buttonPressed && !buttonHeld) {
            buttonHeld = true;
            piston.set(!piston.get());
        } else if (!buttonPressed) {
            buttonHeld = false;
        }
        return buttonHeld;
    }
}