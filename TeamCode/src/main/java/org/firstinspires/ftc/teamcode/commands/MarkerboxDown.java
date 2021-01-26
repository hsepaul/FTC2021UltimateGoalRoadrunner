package org.firstinspires.ftc.teamcode.commands;


/**
 * Created by HPaul on 10/22/2017.
 */

public class MarkerboxDown extends BasicCommand {
    long timeOut;
    boolean markerboxDown = false;


    public MarkerboxDown(){

    }

    public void init() {
        timeOut = System.currentTimeMillis() + 500;
        markerboxDown = false;}

    public void execute(){
        telemetry.addData("Mode:", "Markerbox Down");
        //io.markerBoxDown();
        markerboxDown = true;
    }

    public boolean isFinished(){
        return markerboxDown && System.currentTimeMillis() >= timeOut;
    }
    public void stop() {
        //io.setDrivePower(0,0);
        //io.forkLiftMotor.setPower(0);
    }

}

