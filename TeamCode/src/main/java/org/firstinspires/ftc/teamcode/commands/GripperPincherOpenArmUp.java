package org.firstinspires.ftc.teamcode.commands;


/**
 * Created by HPaul on 10/22/2017.
 */

public class GripperPincherOpenArmUp extends BasicCommand {
    long timeOut;
    long wakeupTime;

    boolean gripperPincherOpen = false;
    boolean gripperPincher2Open = false;


    public GripperPincherOpenArmUp(long timeOut){ this.timeOut = timeOut; }

    public void init() {
        wakeupTime = System.currentTimeMillis() + timeOut;
        //timeOut = System.currentTimeMillis() + 3000;
        gripperPincherOpen = false;
        gripperPincher2Open = false;
    }

    public void execute(){
        telemetry.addData("Mode:", "Gripper Pincher Open Arm Up");
        io.gripperPincherOpen();
        io.gripperPincher2Open();
        gripperPincherOpen = true;
        gripperPincher2Open = true;

        if (io.getArmAngleEncoder() <= -8000){
            io.armAngleMotor.setPower(0);
        } else {
            io.armAngleMotor.setPower(-1);
        }
    }

    public boolean isFinished(){
        if (System.currentTimeMillis() >= wakeupTime) {
            io.armAngleMotor.setPower(0);
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

