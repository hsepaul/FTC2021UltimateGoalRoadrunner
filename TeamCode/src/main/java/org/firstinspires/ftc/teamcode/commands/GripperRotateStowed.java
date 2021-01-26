package org.firstinspires.ftc.teamcode.commands;


/**
 * Created by HPaul on 10/22/2017.
 */

public class GripperRotateStowed extends BasicCommand {
    long timeOut;
    long wakeupTime;

    boolean gripperRotateStowed = false;


    public GripperRotateStowed(long timeOut){ this.timeOut = timeOut; }

    public void init() {
        wakeupTime = System.currentTimeMillis() + timeOut;
        //timeOut = System.currentTimeMillis() + 3000;
        gripperRotateStowed = false;
    }

    public void execute(){
        telemetry.addData("Mode:", "Gripper Pincher Closedl");
        io.gripperRotateStowed();
        gripperRotateStowed = true;
    }

    public boolean isFinished(){
        return gripperRotateStowed || System.currentTimeMillis() >= wakeupTime;
    }
    public void stop() {
        io.setDrivePower(0,0, 0, 0);
        //io.forkLiftMotor.setPower(0);
    }

}

