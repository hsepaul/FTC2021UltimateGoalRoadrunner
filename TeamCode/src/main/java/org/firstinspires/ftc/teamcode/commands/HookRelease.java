package org.firstinspires.ftc.teamcode.commands;


/**
 * Created by HPaul on 10/22/2017.
 */

public class HookRelease extends BasicCommand {
    long timeOut;
    boolean hookRelease = false;


    public HookRelease(){

    }

    public void init() {
        timeOut = System.currentTimeMillis() + 500;
        hookRelease = false;}

    public void execute(){
        telemetry.addData("Mode:", "Hook Release");
        //io.hook.setPower(-.79);
        hookRelease = true;
    }

    public boolean isFinished(){
        return hookRelease && System.currentTimeMillis() >= timeOut;
    }
    public void stop() {
        //io.hookStop();
        //io.setDrivePower(0,0);
        //io.forkLiftMotor.setPower(0);
    }

}

