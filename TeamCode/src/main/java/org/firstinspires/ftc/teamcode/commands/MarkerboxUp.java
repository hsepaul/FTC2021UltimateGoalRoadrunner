package org.firstinspires.ftc.teamcode.commands;


/**
 * Created by HPaul on 10/22/2017.
 */

public class MarkerboxUp extends BasicCommand {
    long timeOut;
    boolean markerboxUp = false;


    public MarkerboxUp(){

    }

    public void init() {
        timeOut = System.currentTimeMillis() + 500;
        markerboxUp = false;}

    public void execute(){
        telemetry.addData("Mode:", "Markerbox Up");
        //io.markerBoxUp();
        markerboxUp = true;
    }

    public boolean isFinished(){
        return markerboxUp && System.currentTimeMillis() >= timeOut;
    }
    public void stop() {
        //io.setDrivePower(0,0);
        //io.forkLiftMotor.setPower(0);
    }

}

