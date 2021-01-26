package org.firstinspires.ftc.teamcode.commands;


/**
 * Created by HPaul on 10/22/2017.
 */

public class ChinDown extends BasicCommand {
    long timeOut;


    public ChinDown(){

    }

    public void init() {
        timeOut = System.currentTimeMillis() + 7000;
        //io.resetDriveEncoders();
    }

    public void execute(){
        telemetry.addData("Mode:", "Chin Down");
        io.chinMotor.setPower(-.75);
    }

    public boolean isFinished(){
        telemetry.addData("Chin Motor Encoder",  "Starting at %.2f",
                io.getChinMotorEncoder());

        return io.getChinMotorEncoder() <= 0 || (io.touchChin.getState() == false) || System.currentTimeMillis() >= timeOut;
    }
    public void stop() {
        io.chinMotor.setPower(0);
    }

}

