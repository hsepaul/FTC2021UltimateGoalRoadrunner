package org.firstinspires.ftc.teamcode.commands;


/**
 * Created by HPaul on 10/22/2017.
 */

public class ElevatorDown extends BasicCommand {
    long timeOut;


    public ElevatorDown(){

    }

    public void init() {
        timeOut = System.currentTimeMillis() + 3000;
        io.resetDriveEncoders();
    }

    public void execute(){
        telemetry.addData("Mode:", "Elevator Down");
        //io.forkLiftMotor.setPower(.6);
    }

    public boolean isFinished(){
        //return io.touchBottom.getState() == false || System.currentTimeMillis() >= timeOut;
        return true;
    }

    public void stop() {
        io.setDrivePower(0,0, 0, 0);
        //io.forkLiftMotor.setPower(0);
    }

}

