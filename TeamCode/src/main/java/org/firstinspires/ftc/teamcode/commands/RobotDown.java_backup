package org.firstinspires.ftc.teamcode.commands;


/**
 * Created by HPaul on 10/22/2017.
 */

public class RobotDown extends BasicCommand {
    long timeOut;


    public RobotDown(){

    }

    public void init() {
        timeOut = System.currentTimeMillis() + 7000;
        io.resetDriveEncoders();
    }

    public void execute(){
        telemetry.addData("Mode:", "Robot Down");
        io.chinMotor.setPower(.75);
    }

    public boolean isFinished(){
        telemetry.addData("Chin Motor Encoder",  "Starting at %.2f",
                io.getChinMotorEncoder());
        return io.getChinMotorEncoder() >= 3550 || System.currentTimeMillis() >= timeOut;
    }
    public void stop() {
        io.chinMotor.setPower(0);
    }

}

