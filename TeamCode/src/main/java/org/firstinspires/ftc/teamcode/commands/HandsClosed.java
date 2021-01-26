package org.firstinspires.ftc.teamcode.commands;


/**
 * Created by HPaul on 10/22/2017.
 */

public class HandsClosed extends BasicCommand {
    long timeOut;

    boolean handsClosed = false;


    public HandsClosed(){

    }

    public void init() {
        timeOut = System.currentTimeMillis() + 3000;
        handsClosed = false;
    }

    public void execute(){
        telemetry.addData("Mode:", "Hands Closed");
        //io.closeHands();
        handsClosed = true;
    }

    public boolean isFinished(){
        return handsClosed || System.currentTimeMillis() >= timeOut;
    }
    public void stop() {
        io.setDrivePower(0,0, 0, 0);
        //io.forkLiftMotor.setPower(0);
    }

}

