package org.firstinspires.ftc.teamcode.commands;


/**
 * Created by HPaul on 10/22/2017.
 */

public class ArmAngleDown1000 extends BasicCommand {
    long timeOut;
    long wakeupTime;

    public ArmAngleDown1000(long timeOut){ this.timeOut = timeOut; }

    public void init() {
        wakeupTime = System.currentTimeMillis() + timeOut;
        //timeOut = System.currentTimeMillis() + 5000;
        io.resetDriveEncoders();
    }

    public void execute(){
        telemetry.addData("Mode:", "Arm Angle Down 1000");
        io.armAngleMotor.setPower(1);
    }

    public boolean isFinished(){
        return io.getArmAngleEncoder() >= 450 || System.currentTimeMillis() >= wakeupTime;
    }
    public void stop() {
        io.setDrivePower(0,0, 0,0);
        io.armAngleMotor.setPower(0);
    }

}

