package com.subsystem;

import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.*;

public class Drive extends Subsystem {
	private Talon talon0;
	private Talon talon1;
	private Talon talon2;
	private Talon talon3;
	private RobotDrive myDrive;
	private Joystick ps3;

	//private AnalogInput in = new AnalogInput(0);
	private SuperiorGyro gyro;
	private double gyroOffset, gyroJoyOffset, targetAngle;
	private static final double NORTH_DIRECTION = 0, EAST_DIRECTION = 90,
			SOUTH_DIRECTION = 180, WEST_DIRECTION = -90;
	private static final double GYRO_OFFSET_CONVERSION = 32.0;
    private Timer resetTimer;
    private boolean isOrienting;
    private static final double ANGLE_THRESHOLD = 1.0;
    private boolean isStrafing;
    private static final int GYRO_PORT = 0;

	public Drive() {
		ps3 = new Joystick(0);
		gyro = new SuperiorGyro(GYRO_PORT, 5);
		talon0 = new Talon(0);
		talon1 = new Talon(1);
		talon2 = new Talon(2);
		talon3 = new Talon(3);
		myDrive = new RobotDrive(talon0, talon1, talon2, talon3);
		myDrive.setInvertedMotor(RobotDrive.MotorType.kFrontRight, true);
		myDrive.setInvertedMotor(RobotDrive.MotorType.kRearRight, true);
		myDrive.setSafetyEnabled(false);
		resetTimer = new Timer();
	}

	private double xyToR(double x, double y){
		return Math.sqrt(x*x + y*y);
	}
	
	private double xyToTheta(double x, double y){
		return (180/Math.PI)*Math.atan2(x, -y);
	}
	
	public void drive() { //called in teleop. 
		double x = ps3.getRawAxis(0);
		double y = ps3.getRawAxis(1);
		
		System.out.println(talon0.get());
		
		if (ps3.getRawButton(5) && ps3.getRawButton(6)) { //pull out
			pullOut();
		} else if (false){ //isStrafing
			myDrive.mecanumDrive_Polar(0.4, 90.0,0.0);
		} else if (stick2.getRawButton(6)){ //Emergency Polar
			myDrive.mecanumDrive_Cartesian(transformJoyInput(x), transformJoyInput(y), ps3.getRawAxis(2), 0.0);
			//myDrive.mecanumDrive_Polar(xyToR(x, y), xyToTheta(x, y), ps3.getRawAxis(2));			
		} else if (ps3.getPOV() != -1){
			myDrive.mecanumDrive_Polar((ps3.getPOV() != 0.0 && ps3.getPOV() != 180) ? 0.6 : 0.4, ps3.getPOV(), manageOrientation(false));
		}
		else {
			//myDrive.mecanumDrive_Cartesian(transformJoyInput(x), transformJoyInput(y), manageOrientation(false), gyroAngle()); //actual drive
			myDrive.mecanumDrive_Cartesian(transformJoyInput(x), transformJoyInput(y), manageOrientation(false), 0.0); //robot oriented with angle manager
		}
		//auto orients
		if (ps3.getRawButton(4)) {
			autoOrient(NORTH_DIRECTION);
		} else if (ps3.getRawButton(3)) {
			autoOrient(EAST_DIRECTION);
		} else if (ps3.getRawButton(2)) {
			autoOrient(SOUTH_DIRECTION);
		} else if (ps3.getRawButton(1)) {
			autoOrient(WEST_DIRECTION);
		}
		adjustGyro(); //drive "tracking" offset
		resetGyroManager(); //reset gyro buttons
	}
	
	public void pullOut(){
		myDrive.tankDrive(-0.7, 0.7);
	}
	
	public void resetGyroManager(){
		if (stick2.getRawButton(2) || stick3.getRawButton(9)){ //hard reset. (reset voltage and angle)
			resetGyro(3);
		} else if (stick2.getRawButton(3) || stick3.getRawButton(10)){ //soft reset. (just zero angle)
			resetGyro(0);
		}
	}

	public void drive(double x, double y, boolean stop) { //drive for auto
		//System.out.println(manageOrientation(true));
		if (stop){
			myDrive.mecanumDrive_Cartesian(0, 0, 0, 0);
		} else if (isStrafing){ //isStrafing
				myDrive.mecanumDrive_Polar(0.4, 90.0,0.0);
		} else {
			myDrive.mecanumDrive_Cartesian(x, -y, manageOrientation(true), gyroAngle());
		}
		//myDrive.mecanumDrive_Polar(x, -y, manageOrientation(true));
	}

	private double manageOrientation(boolean isAuto) { //controls angle for drive. 
		double currAngle = to180(gyroAngle()); // -180 to 180
		
	    if (Math.abs(ps3.getRawAxis(2)) > .1) { //manual mode
	    	targetAngle = to180(gyroAngle());
	        return transformJoyInput(ps3.getRawAxis(2));
	    }
	    
	    double diff = to180(to360(targetAngle - currAngle));//normalize difference (pick shortest turn)
	        
	    if (isOrienting){ //faster
	    	isOrienting = Math.abs(diff) >= ANGLE_THRESHOLD; //might overshoot (shut off and then roll past), but the slow equation will catch it
	        return (0.23 * Math.pow(Math.abs(diff), 0.15)) * (diff < 0 ? -1 : 1);// FAST equation, angular difference to speed
	        //return ((0.56 - 0.19)*Math.sin(Math.PI*Math.abs(diff)/360) + 0.19)*(diff < 0 ? -1 : 1);
	    }
	        
	    if (Math.abs(diff) > ANGLE_THRESHOLD && ((Math.abs(ps3.getRawAxis(0)) > .1 || Math.abs(ps3.getRawAxis(1)) > .1) || isAuto)) { //angle correction, rotate back to target if bigger then 2 degree diff. Also only when it is moving so it doesnt move on its own.  
	    	return 0.072933705074 * Math.pow(Math.abs(diff), 0.436626444302) * (diff < 0 ? -1 : 1);// SLOW equation, angular difference to speed
	    	//return ((0.50 - 0.19)*Math.sin(Math.PI*Math.abs(diff)/360) + 0.19)*(diff < 0 ? -1 : 1);
	    }
	    return 0.0;
	    }

	
    public boolean autoOrient(double angle) { // Sets target. angle in 0 to 360
        targetAngle = to180(angle); // convert to -180 through 180
        isOrienting = true;
        return targetAngle <= to180(gyroAngle()) + ANGLE_THRESHOLD || targetAngle >= to180(gyroAngle()) - ANGLE_THRESHOLD;
    }

	public double gyroAngle() { //0 to 360
		return to360(gyro.getAngle() + gyroOffset + gyroJoyOffset);
	}
	public double getTargetAngle(){
		return to360(targetAngle);
	}

    public void resetGyro(int level) { //gets new velocity voltage and can rezero angle. 
        if (resetTimer.get() == 0.0 || resetTimer.get() > 0.5) { //delay so it doesnt reset it continuously.
            resetTimer.reset();
            resetTimer.start();
            switch(level){
            case 0:
            	gyro.reset();
            	gyroOffset = 0.0;
            	targetAngle = 0.0;
            	break;
            case 1:
            	gyro.free();
                gyro = new SuperiorGyro(GYRO_PORT, 0.5);
                gyroOffset = 0.0;
                targetAngle = 0.0;
                break;
            case 2: // maintain heading
            	gyroOffset = gyroAngle();
            	gyro.free();
                gyro = new SuperiorGyro(GYRO_PORT, 1.0);
                clearTarget();
                break;
	        case 3: // new heading
	        	gyro.free();
	            gyro = new SuperiorGyro(GYRO_PORT, 1.0);
	            gyroOffset = 0.0;
	            targetAngle = 0.0;
	            break;
            }
        }
    }

	private double transformJoyInput(double input) { //desensitize joysticks. 
		//y=-.099a+.1,  a=-1.(01)
		//y=-.192a+.2,  a=0.5208(3)
		final double DEAD_ZONE = 0.08, JOY_SCALE = 0.5208333;
		if(Math.abs(input) < DEAD_ZONE){
			return 0;
		}
		//return (JOY_SCALE)*Math.pow(input, 3) + (1-JOY_SCALE)*input;
		return (-0.4414 * Math.pow(Math.abs(input), 2) + 1.18248 * Math.abs(input) - 0.07431) * (input < 0.0 ? -1 : 1);
	}

	public double to360(double in) {
		in %= 360;
		return in + (in < 0 ? 360 : 0);
	}
	
	public double to180(double in){
		if(in > 180) {
			return in - 360;
		} else {
			return in;
		}
	}
	public void clearTarget(){
		targetAngle = to180(gyroAngle());
	}

	private void adjustGyro() { //drive tracking
		gyroJoyOffset = stick2.getRawAxis(1) * GYRO_OFFSET_CONVERSION;
	}
	public void setIsStrafing(boolean strafe){
		isStrafing = strafe;
	}
	public void setGyroOffset(double angle){
		gyroOffset = angle;
	}
	public boolean getIsOrienting() {
		return isOrienting;
	}
}