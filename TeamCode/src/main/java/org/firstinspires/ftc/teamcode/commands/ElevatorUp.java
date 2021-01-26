package org.firstinspires.ftc.teamcode.commands;


/**
 * Created by HPaul on 10/22/2017.
 */

public class ElevatorUp extends BasicCommand {
    long timeOut;


    public ElevatorUp(){

    }

    public void init() {
        timeOut = System.currentTimeMillis() + 3000;
        io.resetDriveEncoders();
    }

    public void execute(){
        telemetry.addData("Mode:", "Elevator Up");
        //io.forkLiftMotor.setPower(-.6);
    }

    public boolean isFinished(){
        //return io.getForkLiftMotorEncoder() <= -1000 || System.currentTimeMillis() >= timeOut;
        return true;
    }
    public void stop() {
        io.setDrivePower(0,0, 0,0);
        //io.forkLiftMotor.setPower(0);
    }

}

