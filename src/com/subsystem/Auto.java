package com.subsystem;

import edu.wpi.first.wpilibj.Timer;

public class Auto extends Subsystem {
	private static final double AUTO_CLAW_DELAY_TIME = 0.8;
	private static final double AUTO_ABANDON_TOTES_TIME = 0.5;
	
	private int autoMode = 0;
	
	private Drive drive;
	private Elevator elevator;
	private Timer autoTimer;
	
	private int autoState;
	
	private double armHeight;
	private double vx, vy;
	
	private boolean isPullingOut = false;

	public Auto(Drive drive, Elevator elevator) {
		this.drive = drive;
		this.elevator = elevator;
		autoTimer = new Timer();
		autoTimer.start();
		//System.out.println(drive.gyroAngle());
	}
	
	public void run(){
		if(isPullingOut){
			drive.pullOut();
		}else {
			if (vx == 0 && vy == 0){
				drive.drive(vx, vy, true);
			} else {
				drive.drive(vx, vy, false);
			}
		}
		elevator.armHandler(armHeight, false);
		switch(autoMode){
		case 0:
			vx = 0;
			vy = 0;
			if(autoTimer.get() > 60){
				autoTimer.reset();
				System.out.println(drive.gyroAngle());
			}
		case 1:
			autoDoSteps(autoDrive);
			break;
		case 2:
			autoDoSteps(auto3TotesWide);
			break;
		case 3:
			autoDoSteps(auto1Tote1CanWide);
			break;
		case 4:
			autoDoSteps(auto3Can);
			break;
		case 5:
			autoDoSteps(auto1Bump);
			break;
		case 6:
			autoDoSteps(auto1ToteWide);
			break;
		case 7:
			autoDoSteps(auto1CanWide);
			break;
		default:
			System.out.println("Robots don't quit!!!!!");
		}
	}
	
	private void autoDoSteps(AutoStep[] steps){//Perform the steps in a list of autoSteps
		if(isPreviousStepRunning(steps)){//Check if the previous step is still running.  Also manages previous step.
			return;
		}//if the previous step is finished, start the next step, then reset the timer.
		AutoStep step = steps[autoState];
		switch(step.action){
		case DRIVE_NORTH:
			vx = 0;
			vy = 0.7;
			break;
		case DRIVE_EAST:
			vx = 0.9;
			vy = 0;
			break;
		case DRIVE_SOUTH:
			vx = 0;
			vy = -0.7;
			break;
		case DRIVE_WEST:
			vx = -0.9;
			vy = 0;
			break;
		case PULL_OUT:
			drive.pullOut();
			isPullingOut = true;
			break;
		case STOP_DRIVING:
			vx = 0;
			vy = 0;
			break;
		case TURN:
			drive.autoOrient(step.duration);//duration serves as the direction as well
			break;
		case GRAB_TOTE:
			elevator.setClaw(true, true);
			break;
		case GRAB_WIDE:
			elevator.setClaw(true, false);
			break;
		case GRAB_BIN:
			elevator.setClaw(true, true);//Not sure
			break;
		case MOVE_ARM:
			armHeight = step.duration;//duration serves as the height as well
			//the arm is moved to the current armHeight in run
			break;
		case WAIT_ARM://WAIT_ARM does not do anything except wait (look in isPreviousStepRunning)
			break;
		case RELEASE:
			elevator.setClaw(false, false);
			break;
		case STOP:
			drive.resetGyro(2);
			break;
		}
		autoTimer.reset();
		autoState++;
	}

	private boolean isPreviousStepRunning(AutoStep[] steps){
		if(autoState == 0){
			return false;
		}
		AutoStep step = steps[autoState - 1];
		switch(step.action){
		case MOVE_ARM:
			return false;
		case WAIT_ARM://MOVE_ARM does not wait for the arm to reach its position: use this instead
			return !elevator.armHandler(armHeight, true);
		case TURN://TURN requires a function to be called every cycle.
			//TODO: Does this need to be called once they are done to stop?  If so, move these to the main do auto function.
			return !drive.autoOrient(step.duration);//duration means direction here.
		case STOP://STOP can't finish
			armHeight = -1; //WILL TURN OFF MOTORS at end. will be a PROBLEM
			return true;
		default://move on when timer is done
			if(autoTimer.get() < step.duration){
				return true;
			}
			if(step.action == AutoAction.PULL_OUT){
				isPullingOut = false;
			}
			return false;
		}
	}
	
	private AutoStep[] autoDrive = {//Start facing North
		new AutoStep(AutoAction.MOVE_ARM, -1),
		new AutoStep(AutoAction.DRIVE_NORTH, 1.6),
		new AutoStep(AutoAction.STOP_DRIVING, 0),
		new AutoStep(AutoAction.STOP, 0)
	};
	
	private AutoStep[] auto3TotesWide = { //Start facing South
			new AutoStep(AutoAction.MOVE_ARM, -1),
			new AutoStep(AutoAction.GRAB_TOTE, AUTO_CLAW_DELAY_TIME),
			new AutoStep(AutoAction.STOP_DRIVING, 2.0),//0
			new AutoStep(AutoAction.MOVE_ARM, ARM_3rd_TOTE_HEIGHT),
			new AutoStep(AutoAction.DRIVE_WEST, 0.8),
			new AutoStep(AutoAction.MOVE_ARM, ARM_2nd_TOTE_HEIGHT),
			new AutoStep(AutoAction.WAIT_ARM, 0),
			new AutoStep(AutoAction.STOP_DRIVING, 0.4),//0
			new AutoStep(AutoAction.RELEASE, AUTO_CLAW_DELAY_TIME),
			new AutoStep(AutoAction.MOVE_ARM, ARM_1st_TOTE_HEIGHT),
			new AutoStep(AutoAction.WAIT_ARM, 0),
			new AutoStep(AutoAction.GRAB_TOTE, AUTO_CLAW_DELAY_TIME),
			new AutoStep(AutoAction.MOVE_ARM, ARM_3rd_TOTE_HEIGHT),
			new AutoStep(AutoAction.DRIVE_WEST, 0.9),
			new AutoStep(AutoAction.STOP_DRIVING, 0.4),//0
			new AutoStep(AutoAction.MOVE_ARM, ARM_2nd_TOTE_HEIGHT),
			new AutoStep(AutoAction.WAIT_ARM, 0),
			new AutoStep(AutoAction.RELEASE, AUTO_CLAW_DELAY_TIME),
			new AutoStep(AutoAction.MOVE_ARM, ARM_1st_TOTE_HEIGHT),
			new AutoStep(AutoAction.WAIT_ARM, 0),
			new AutoStep(AutoAction.GRAB_TOTE, AUTO_CLAW_DELAY_TIME),
			new AutoStep(AutoAction.MOVE_ARM, ARM_DRIVE_HEIGHT),
			new AutoStep(AutoAction.TURN, 0.0),
			new AutoStep(AutoAction.STOP_DRIVING, 0.01),//0
			new AutoStep(AutoAction.DRIVE_NORTH, 2.0), //1.5
			new AutoStep(AutoAction.STOP_DRIVING, 0.0), //0
			new AutoStep(AutoAction.MOVE_ARM, ARM_1st_TOTE_HEIGHT),
			new AutoStep(AutoAction.WAIT_ARM, 0),
			new AutoStep(AutoAction.RELEASE, AUTO_CLAW_DELAY_TIME),
			new AutoStep(AutoAction.MOVE_ARM, ARM_OVER_CAN_HEIGHT),
			new AutoStep(AutoAction.WAIT_ARM, 0),
			new AutoStep(AutoAction.MOVE_ARM, -1),
			new AutoStep(AutoAction.STOP, 0)
	};
	
	private AutoStep[] auto1Tote1CanWide = { //Start facing North
			new AutoStep(AutoAction.MOVE_ARM, -1),
			new AutoStep(AutoAction.GRAB_WIDE, AUTO_CLAW_DELAY_TIME),
			new AutoStep(AutoAction.DRIVE_EAST, 0.6),
			new AutoStep(AutoAction.DRIVE_NORTH, 1.7),
			new AutoStep(AutoAction.STOP_DRIVING, 0.4),
			new AutoStep(AutoAction.RELEASE, AUTO_CLAW_DELAY_TIME),
			new AutoStep(AutoAction.MOVE_ARM, ARM_3rd_TOTE_HEIGHT),
			new AutoStep(AutoAction.PULL_OUT, 0.2),
			new AutoStep(AutoAction.STOP_DRIVING, 0.1),
			new AutoStep(AutoAction.WAIT_ARM, 0),
			new AutoStep(AutoAction.DRIVE_WEST, 1.25),
			new AutoStep(AutoAction.STOP_DRIVING, 0.2),
			new AutoStep(AutoAction.MOVE_ARM, ARM_1st_TOTE_HEIGHT), //picking up 
			new AutoStep(AutoAction.DRIVE_SOUTH, 1.55),
			new AutoStep(AutoAction.STOP_DRIVING, 0.4),
			new AutoStep(AutoAction.GRAB_TOTE, AUTO_CLAW_DELAY_TIME),
			new AutoStep(AutoAction.MOVE_ARM, ARM_DRIVE_HEIGHT),
			new AutoStep(AutoAction.STOP_DRIVING, 0.5),
			new AutoStep(AutoAction.DRIVE_NORTH, 1.7),
			new AutoStep(AutoAction.STOP_DRIVING, 0.4),
			new AutoStep(AutoAction.RELEASE, AUTO_CLAW_DELAY_TIME),
			new AutoStep(AutoAction.MOVE_ARM, ARM_OVER_CAN_HEIGHT),
			new AutoStep(AutoAction.WAIT_ARM, 0),
			new AutoStep(AutoAction.MOVE_ARM, -1),
			new AutoStep(AutoAction.STOP_DRIVING, 0),
			new AutoStep(AutoAction.STOP, 0)
	};
	
	private AutoStep[] auto3Can = { //Start facing South
			new AutoStep(AutoAction.MOVE_ARM, -1),
			//new AutoStep(AutoAction.GRAB_CAN, AUTO_GRAB_TOTE_TIME),//Grab can 1
			//new AutoStep(AutoAction.MOVE_ARM, ARM_1st_TOTE_HEIGHT),
			new AutoStep(AutoAction.STOP_DRIVING, 0.2),
			//new AutoStep(AutoAction.DRIVE_WEST, 2.5), //for dodging step, not needed
			new AutoStep(AutoAction.DRIVE_NORTH, .5), //1.5
			new AutoStep(AutoAction.STOP_DRIVING, 0.2),
			//new AutoStep(AutoAction.MOVE_ARM, ARM_1st_TOTE_HEIGHT),
			//new AutoStep(AutoAction.WAIT_ARM, 0),
			//new AutoStep(AutoAction.RELEASE, AUTO_MOVE_TO_TOTE_TIME),
			new AutoStep(AutoAction.PULL_OUT, 0.25),
			new AutoStep(AutoAction.DRIVE_EAST, 2.25),
			new AutoStep(AutoAction.STOP_DRIVING, 0.2),
			new AutoStep(AutoAction.DRIVE_SOUTH, .75), //1.75
			new AutoStep(AutoAction.STOP_DRIVING, 1.2),
			//new AutoStep(AutoAction.MOVE_ARM, ARM_1st_TOTE_HEIGHT),
			//new AutoStep(AutoAction.GRAB_CAN, AUTO_GRAB_TOTE_TIME),//Grab can 2
			//new AutoStep(AutoAction.MOVE_ARM, ARM_1st_TOTE_HEIGHT),
			new AutoStep(AutoAction.DRIVE_NORTH, 1),//1.5
			new AutoStep(AutoAction.STOP_DRIVING, 0.2),
			//new AutoStep(AutoAction.MOVE_ARM, ARM_1st_TOTE_HEIGHT),
			//new AutoStep(AutoAction.WAIT_ARM, 0),
			//new AutoStep(AutoAction.RELEASE, AUTO_MOVE_TO_TOTE_TIME),
			new AutoStep(AutoAction.PULL_OUT, 0.25),
			new AutoStep(AutoAction.DRIVE_EAST, 2.25),
			new AutoStep(AutoAction.STOP_DRIVING, 0.2),
			new AutoStep(AutoAction.DRIVE_SOUTH, .75),//1.75
			new AutoStep(AutoAction.STOP_DRIVING, 1.2),
			//new AutoStep(AutoAction.MOVE_ARM, ARM_1st_TOTE_HEIGHT),
			//new AutoStep(AutoAction.GRAB_CAN, AUTO_GRAB_TOTE_TIME),//Grab can 3
			//new AutoStep(AutoAction.MOVE_ARM, ARM_1st_TOTE_HEIGHT),
			new AutoStep(AutoAction.DRIVE_NORTH, 1),//1.5
			new AutoStep(AutoAction.STOP_DRIVING, 1.2),
			new AutoStep(AutoAction.MOVE_ARM, ARM_2nd_TOTE_HEIGHT),
			new AutoStep(AutoAction.WAIT_ARM, 0),
			new AutoStep(AutoAction.MOVE_ARM, -1),
			new AutoStep(AutoAction.STOP, 0)
	};
	
	public AutoStep[] auto1Bump = {
			new AutoStep(AutoAction.MOVE_ARM, -1),
			new AutoStep(AutoAction.GRAB_TOTE, AUTO_CLAW_DELAY_TIME), //WIDE
			new AutoStep(AutoAction.MOVE_ARM, ARM_DRIVE_HEIGHT),
			//new AutoStep(AutoAction.DRIVE_EAST, 0.5), //1.5
			new AutoStep(AutoAction.STOP_DRIVING, 0.5), //0
			new AutoStep(AutoAction.DRIVE_NORTH, 1.7), //1.5
			new AutoStep(AutoAction.STOP_DRIVING, 0.5), //0
			new AutoStep(AutoAction.MOVE_ARM, ARM_1st_TOTE_HEIGHT),
			new AutoStep(AutoAction.RELEASE, AUTO_CLAW_DELAY_TIME),
			new AutoStep(AutoAction.MOVE_ARM, ARM_OVER_CAN_HEIGHT),
			new AutoStep(AutoAction.WAIT_ARM, 0),
			new AutoStep(AutoAction.MOVE_ARM, -1),
			new AutoStep(AutoAction.STOP_DRIVING, 0),
			new AutoStep(AutoAction.STOP, 0)
	};
	
	public AutoStep[] auto1ToteWide = {
			new AutoStep(AutoAction.MOVE_ARM, -1),
			new AutoStep(AutoAction.GRAB_TOTE, AUTO_CLAW_DELAY_TIME), //WIDE
			new AutoStep(AutoAction.MOVE_ARM, ARM_DRIVE_HEIGHT),
			//new AutoStep(AutoAction.DRIVE_EAST, 0.5), //1.5
			new AutoStep(AutoAction.STOP_DRIVING, 0.5), //0
			new AutoStep(AutoAction.DRIVE_NORTH, 1.5), //1.5
			new AutoStep(AutoAction.STOP_DRIVING, 0.5), //0
			new AutoStep(AutoAction.MOVE_ARM, ARM_1st_TOTE_HEIGHT),
			new AutoStep(AutoAction.RELEASE, AUTO_CLAW_DELAY_TIME),
			new AutoStep(AutoAction.MOVE_ARM, ARM_OVER_CAN_HEIGHT),
			new AutoStep(AutoAction.WAIT_ARM, 0),
			new AutoStep(AutoAction.MOVE_ARM, -1),
			new AutoStep(AutoAction.STOP_DRIVING, 0),
			new AutoStep(AutoAction.STOP, 0)
	};
	public AutoStep[] auto1CanWide = { //NOT SET UP
			new AutoStep(AutoAction.MOVE_ARM, -1),
			new AutoStep(AutoAction.GRAB_TOTE, AUTO_CLAW_DELAY_TIME), //WIDE
			new AutoStep(AutoAction.MOVE_ARM, ARM_2nd_TOTE_HEIGHT),
			new AutoStep(AutoAction.DRIVE_WEST, 1.0), //1.5
			new AutoStep(AutoAction.STOP_DRIVING, 0.5), //0
			new AutoStep(AutoAction.DRIVE_NORTH, 1.7), //1.5
			new AutoStep(AutoAction.STOP_DRIVING, 0.5), //0
			new AutoStep(AutoAction.MOVE_ARM, ARM_1st_TOTE_HEIGHT),
			//new AutoStep(AutoAction.RELEASE, AUTO_CLAW_DELAY_TIME),
			//new AutoStep(AutoAction.MOVE_ARM, ARM_OVER_CAN_HEIGHT),
			new AutoStep(AutoAction.WAIT_ARM, 0),
			new AutoStep(AutoAction.MOVE_ARM, -1),
			new AutoStep(AutoAction.STOP_DRIVING, 0),
			new AutoStep(AutoAction.STOP, 0)
	};
	
	public void setAutoMode(int mode){
		autoMode = mode;
	}
	
	public double getStartAngle(){
		switch(autoMode){
		case 0:
			return 270;
		case 1:
			return 180;
		case 2://3 tote
			return 270.0;
		case 3: //1 tote 1 can
			return 180.0;
		case 4:
			return 180.0;
		case 5:
			return 180.0;
		case 6:
			return 180.0;
		case 7:
			return 180.0;
		default:
			return 0.0;
		}
	}
}

enum AutoAction{//all actions are limited by a time except TURN and MOVE_ARM, which are limited by the target value
	DRIVE_NORTH, DRIVE_EAST, DRIVE_SOUTH, DRIVE_WEST, PULL_OUT, STOP_DRIVING, TURN, GRAB_TOTE, GRAB_WIDE, GRAB_BIN, MOVE_ARM, WAIT_ARM, RELEASE, STOP, WAIT
}

class AutoStep{
	public double duration;
	public AutoAction action;
	
	public AutoStep(AutoAction action, double duration){
		this.duration = duration;
		this.action = action;
	}
}
