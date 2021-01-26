package org.firstinspires.ftc.teamcode.commands;


/**
 * Created by HPaul on 10/22/2017.
 */

public class ArmAngleDownReturnToZero extends BasicCommand {
    long timeOut;
    long wakeupTime;

    public ArmAngleDownReturnToZero(long timeOut){ this.timeOut = timeOut; }

    public void init() {
        wakeupTime = System.currentTimeMillis() + timeOut;
        //timeOut = System.currentTimeMillis() + 5000;
    }

    public void execute(){
        telemetry.addData("Mode:", "Arm Angle Down Return To Zero");
        io.armAngleMotor.setPower(1);
    }

    public boolean isFinished(){
        if (io.getArmAngleEncoder() >= 0 || System.currentTimeMillis() >= wakeupTime) {
            io.armAngleMotor.setPower(0);
        }
        return io.getArmAngleEncoder() >= 0 || System.currentTimeMillis() >= wakeupTime;
    }
    public void stop() {
        io.setDrivePower(0,0, 0,0);
        io.armAngleMotor.setPower(0);
    }

}

