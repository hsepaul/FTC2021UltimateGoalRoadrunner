package org.firstinspires.ftc.teamcode.commands;


/**
 * Created by HPaul on 10/22/2017.
 */

public class HandsOpened extends BasicCommand {
    long timeOut;
    boolean handsOpened = false;


    public HandsOpened(){

    }

    public void init() {
        timeOut = System.currentTimeMillis() + 3000;
        handsOpened = false;
    }

    public void execute(){
        telemetry.addData("Mode:", "Hands Opened");
        //io.retractHands();
        handsOpened = true;
    }

    public boolean isFinished(){
        return handsOpened || System.currentTimeMillis() >= timeOut;
    }
    public void stop() {
        io.setDrivePower(0,0, 0, 0);
        //io.forkLiftMotor.setPower(0);
    }

}

