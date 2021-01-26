package org.firstinspires.ftc.teamcode.commands;


/**
 * Created by HPaul on 10/22/2017.
 */

public class ArmAngleUp3000 extends BasicCommand {
    long timeOut;
    long wakeupTime;

    public ArmAngleUp3000(long timeOut){ this.timeOut = timeOut; }

    public void init() {
        wakeupTime = System.currentTimeMillis() + timeOut;
        //timeOut = System.currentTimeMillis() + 5000;
        io.resetDriveEncoders();
    }

    public void execute(){
        telemetry.addData("Mode:", "Arm Angle Up 3000");
        io.armAngleMotor.setPower(-1);
    }

    public boolean isFinished(){
        return io.getArmAngleEncoder() <= -2850 || System.currentTimeMillis() >= wakeupTime;
    }
    public void stop() {
        io.setDrivePower(0,0, 0,0);
        io.armAngleMotor.setPower(0);
    }

}

