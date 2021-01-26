package org.firstinspires.ftc.teamcode.commands;


/**
 * Created by HPaul on 10/22/2017.
 */

public class GripperRotateSlightlyUp extends BasicCommand {
    long timeOut;
    long wakeupTime;

    boolean gripperRotateSlightlyUp = false;


    public GripperRotateSlightlyUp(long timeOut){ this.timeOut = timeOut; }

    public void init() {
        wakeupTime = System.currentTimeMillis() + timeOut;
        //timeOut = System.currentTimeMillis() + 3000;
        gripperRotateSlightlyUp = false;
    }

    public void execute(){
        telemetry.addData("Mode:", "Gripper Slightly Up");
        io.setGripperRotateSlightlyUp();
        gripperRotateSlightlyUp = true;
    }

    public boolean isFinished(){
        return gripperRotateSlightlyUp || System.currentTimeMillis() >= wakeupTime;
    }
    public void stop() {
        io.setDrivePower(0,0, 0, 0);
        //io.forkLiftMotor.setPower(0);
    }

}

