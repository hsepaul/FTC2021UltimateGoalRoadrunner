package org.firstinspires.ftc.teamcode.commands;


/**
 * Created by HPaul on 10/22/2017.
 */

public class GripperPincherStopped extends BasicCommand {
    long timeOut;
    long wakeupTime;

    boolean gripperPincherStopped = false;
    boolean gripperPincher2Stopped = false;


    public GripperPincherStopped(long timeOut){ this.timeOut = timeOut; }

    public void init() {
        wakeupTime = System.currentTimeMillis() + timeOut;
        //timeOut = System.currentTimeMillis() + 3000;
        gripperPincherStopped = false;
        gripperPincher2Stopped = false;
    }

    public void execute(){
        telemetry.addData("Mode:", "Gripper Pincher Stopped");
        io.gripperPincherStopped();
        io.gripperPincher2Stopped();
        gripperPincherStopped = true;
        gripperPincher2Stopped = true;
    }

    public boolean isFinished(){
        return gripperPincherStopped || gripperPincher2Stopped || System.currentTimeMillis() >= wakeupTime;
    }
    public void stop() {
        io.setDrivePower(0,0, 0, 0);
        //io.forkLiftMotor.setPower(0);
    }

}

