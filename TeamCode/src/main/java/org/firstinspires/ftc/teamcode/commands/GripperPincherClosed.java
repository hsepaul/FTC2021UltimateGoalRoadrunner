package org.firstinspires.ftc.teamcode.commands;


/**
 * Created by HPaul on 10/22/2017.
 */

public class GripperPincherClosed extends BasicCommand {
    long timeOut;
    long wakeupTime;

    boolean gripperPincherClosed = false;
    boolean gripperPincher2Closed = false;


    public GripperPincherClosed(long timeOut){ this.timeOut = timeOut; }

    public void init() {
        wakeupTime = System.currentTimeMillis() + timeOut;
        //timeOut = System.currentTimeMillis() + 3000;
        gripperPincherClosed = false;
        gripperPincher2Closed = false;
    }

    public void execute(){
        telemetry.addData("Mode:", "Gripper Pincher Closed");
        io.gripperPincherClosed();
        io.gripperPincher2Closed();
        gripperPincherClosed = true;
        gripperPincher2Closed = true;
    }

    public boolean isFinished(){
        if (System.currentTimeMillis() >= wakeupTime) {
            io.gripperPincherStopped();
            io.gripperPincher2Stopped();
        }
        return System.currentTimeMillis() >= wakeupTime;
        //return gripperPincherClosed || gripperPincher2Closed || System.currentTimeMillis() >= wakeupTime;
    }
    public void stop() {
        io.setDrivePower(0,0, 0, 0);
        //io.forkLiftMotor.setPower(0);
    }

}

