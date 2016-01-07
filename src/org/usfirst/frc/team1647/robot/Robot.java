package org.usfirst.frc.team1647.robot;

import com.subsystem.Auto;
import com.subsystem.Drive;
import com.subsystem.Elevator;
import com.subsystem.Messenger;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Timer;

public class Robot extends SampleRobot {
	private Drive drive = new Drive();
	private Elevator elevator = new Elevator();
	private Messenger messenger = new Messenger();
	private Auto auto = new Auto(drive, elevator);
    private Compressor air = new Compressor();
    private DigitalInput auto0 = new DigitalInput(0);
    private DigitalInput auto1 = new DigitalInput(1);
    private DigitalInput auto2 = new DigitalInput(2);
    private DigitalInput auto3 = new DigitalInput(3);
    public void robotInit(){
    	air.start();
    }
    
	public void autonomous() {
		auto.setAutoMode(calculateAutoMode(false, auto2.get(), auto1.get(), auto0.get())); //3- 1 & 1;  6 - 1can
		drive.resetGyro(1);
		drive.setGyroOffset(auto.getStartAngle());
		drive.clearTarget();
		while (isAutonomous()) {
			messenger.setData(drive.gyroAngle(), elevator.armHandler(), drive.getTargetAngle(), drive.getIsOrienting(), elevator.getPot(), elevator.getTargetVoltage());
			messenger.putData();
			auto.run();
			drive.setIsStrafing(elevator.manageStrafe());
		}
	}

	public void operatorControl() {
		while (isOperatorControl()) {
			System.out.println(calculateAutoMode(false, auto2.get(), auto1.get(), auto0.get()));
			drive.drive();
			messenger.setData(drive.gyroAngle(), elevator.armHandler(), drive.getTargetAngle(), drive.getIsOrienting(), elevator.getPot(), elevator.getTargetVoltage());
			messenger.putData();
			elevator.setClaw();
			drive.setIsStrafing(elevator.manageStrafe());
		}
	}

	public void test() {
	}
	
    public int binaryToInt(String input) { //converts to binary to decimal. Only 4 numbers
        int output = 0;

        if (input.substring(3).equals("1")) {
            output += 1;
        }
        if (input.substring(2, 3).equals("1")) {
            output += 2;
        }
        if (input.substring(1, 2).equals("1")) {
            output += 4;
        }
        if (input.substring(0, 1).equals("1")) {
            output += 8;
        }

        return output;
    }

    public int calculateAutoMode(boolean auto1, boolean auto2, boolean auto3, boolean auto4) {
        int autoMode = 0;
        String binaryBuilder = "";
        //builds binary number
        binaryBuilder += auto1 ? "1" : "0";
        binaryBuilder += auto2 ? "1" : "0";
        binaryBuilder += auto3 ? "1" : "0";
        binaryBuilder += auto4 ? "1" : "0";
        autoMode = binaryToInt(binaryBuilder);
        return autoMode;
    }
}