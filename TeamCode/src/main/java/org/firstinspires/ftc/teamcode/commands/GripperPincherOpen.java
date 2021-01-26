package org.firstinspires.ftc.teamcode.commands;


/**
 * Created by HPaul on 10/22/2017.
 */

public class GripperPincherOpen extends BasicCommand {
    long timeOut;
    long wakeupTime;

    boolean gripperPincherOpen = false;
    boolean gripperPincher2Open = false;


    public GripperPincherOpen(long timeOut){ this.timeOut = timeOut; }

    public void init() {
        wakeupTime = System.currentTimeMillis() + timeOut;
        //timeOut = System.currentTimeMillis() + 3000;
        gripperPincherOpen = false;
        gripperPincher2Open = false;
    }

    public void execute(){
        telemetry.addData("Mode:", "Gripper Pincher Open");
        io.gripperPincherOpen();
        io.gripperPincher2Open();
        gripperPincherOpen = true;
        gripperPincher2Open = true;
    }

    public boolean isFinished(){
        if (System.currentTimeMillis() >= wakeupTime) {
            io.gripperPincherStopped();
            io.gripperPincher2Stopped();
        }
        return System.currentTimeMillis() >= wakeupTime;
        //return gripperPincherOpen || gripperPincher2Open|| System.currentTimeMillis() >= wakeupTime;
    }
    public void stop() {
        io.setDrivePower(0,0, 0, 0);
        //io.forkLiftMotor.setPower(0);
    }

}

