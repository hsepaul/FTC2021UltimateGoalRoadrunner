package org.firstinspires.ftc.teamcode.commands;


/**
 * Created by HPaul on 10/22/2017.
 */

public class ReleaseCage extends BasicCommand {
    long timeOut;
    boolean releaseCage = false;


    public ReleaseCage(){

    }

    public void init() {
        timeOut = System.currentTimeMillis() + 1500;
        releaseCage = false;}

    public void execute(){
        telemetry.addData("Mode:", "Release Cage");
        io.domExtendMotor.setPower(1);
        releaseCage = true;
    }

    public boolean isFinished(){
        return releaseCage && System.currentTimeMillis() >= timeOut;
    }
    public void stop() {
        io.domExtendMotor.setPower(0);
    }

}

