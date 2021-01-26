package org.firstinspires.ftc.teamcode.commands;


/**
 * Created by HPaul on 10/22/2017.
 */

public class MarkerboxFlat extends BasicCommand {
    long timeOut;
    boolean markerboxFlat = false;


    public MarkerboxFlat(){

    }

    public void init() {
        timeOut = System.currentTimeMillis() + 500;
        markerboxFlat = false;}

    public void execute(){
        telemetry.addData("Mode:", "Markerbox Flat");
        //io.markerBoxFlat();
        markerboxFlat = true;
    }

    public boolean isFinished(){
        return markerboxFlat && System.currentTimeMillis() >= timeOut;
    }
    public void stop() {
        //io.setDrivePower(0,0);
        //io.forkLiftMotor.setPower(0);
    }

}

