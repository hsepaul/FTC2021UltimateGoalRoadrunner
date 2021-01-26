package org.firstinspires.ftc.teamcode.commands;


/**
 * Created by HPaul on 10/22/2017.
 */

public class CageHome extends BasicCommand {
    long timeOut;
    boolean cageHome = false;


    public CageHome(){

    }

    public void init() {
        timeOut = System.currentTimeMillis() + 1500;
        cageHome = false;}

    public void execute(){
        telemetry.addData("Mode:", "Cage Home");
        //io.domExtendMotor.setPower(-1);
        cageHome = true;
    }

    public boolean isFinished(){
        return cageHome && System.currentTimeMillis() >= timeOut;
    }
    public void stop() {
        //io.domExtendMotor.setPower(0);
    }

}

