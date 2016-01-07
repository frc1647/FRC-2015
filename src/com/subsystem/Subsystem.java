package com.subsystem;

import edu.wpi.first.wpilibj.Joystick;

public class Subsystem {
	protected static Joystick stick2;
	protected static Joystick stick3;
	protected static final double ARM_PRESSURE_HEIGHT = 4.3;
	protected static final double ARM_3rd_TOTE_HEIGHT = 3.1;
	protected static final double ARM_2nd_TOTE_HEIGHT = 3.724;
	protected static final double ARM_1st_TOTE_HEIGHT = 4.403;
	protected static final double ARM_OVER_CAN_HEIGHT = 2.5; 
	protected static final double ARM_DRIVE_HEIGHT = 4.2; 
	protected static final double ARM_LOW_DRIVE_HEIGHT = 4.3; 
	public Subsystem() {
		stick2 = new Joystick(1); // custom
		stick3 = new Joystick(2); // fail safe
	}
	
    double linearTransform (double in, double inStart, double inEnd, double outStart, double outEnd){
  	  return (in - inStart)*(outEnd - outStart)/(inEnd - inStart) + outStart;
  	}

    
}