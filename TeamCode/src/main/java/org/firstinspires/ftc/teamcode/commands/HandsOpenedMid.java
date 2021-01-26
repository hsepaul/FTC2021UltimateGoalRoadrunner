package org.firstinspires.ftc.teamcode.commands;


/**
 * Created by HPaul on 10/22/2017.
 */

public class HandsOpenedMid extends BasicCommand {
    long timeOut;
    boolean handsOpenedMid = false;


    public HandsOpenedMid(){

    }

    public void init() {
        timeOut = System.currentTimeMillis() + 3000;
        handsOpenedMid = false;}

    public void execute(){
        telemetry.addData("Mode:", "Hands Opened Mid");
        //io.retractHandsMid();
        handsOpenedMid = true;
    }

    public boolean isFinished(){
        return handsOpenedMid || System.currentTimeMillis() >= timeOut;
    }
    public void stop() {
        io.setDrivePower(0,0, 0, 0);
        //io.forkLiftMotor.setPower(0);
    }

}

