package org.firstinspires.ftc.teamcode.commands;


/**
 * Created by HPaul on 10/22/2017.
 */

public class ArmAngleUp extends BasicCommand {
    long timeOut;
    long wakeupTime;

    public ArmAngleUp(long timeOut){ this.timeOut = timeOut; }

    public void init() {
        wakeupTime = System.currentTimeMillis() + timeOut;
        //timeOut = System.currentTimeMillis() + 5000;
        io.resetDriveEncoders();
    }

    public void execute(){
        telemetry.addData("Mode:", "Arm Angle Up");
        io.armAngleMotor.setPower(-1);
    }

    public boolean isFinished(){
        return io.getArmAngleEncoder() <= -2000 || System.currentTimeMillis() >= wakeupTime;
    }
    public void stop() {
        io.setDrivePower(0,0, 0,0);
        io.armAngleMotor.setPower(0);
    }

}

