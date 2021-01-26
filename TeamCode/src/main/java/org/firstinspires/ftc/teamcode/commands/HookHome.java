package org.firstinspires.ftc.teamcode.commands;


/**
 * Created by HPaul on 10/22/2017.
 */

public class HookHome extends BasicCommand {
    long timeOut;
    boolean hookHome = false;


    public HookHome(){

    }

    public void init() {
        timeOut = System.currentTimeMillis() + 500;
        hookHome = false;}

    public void execute(){
        telemetry.addData("Mode:", "Hook Home");
        //io.hook.setPower(.79);
        hookHome = true;
    }

    public boolean isFinished(){
        return hookHome && System.currentTimeMillis() >= timeOut;
    }
    public void stop() {
        //io.hookStop();
        //io.setDrivePower(0,0);
        //io.forkLiftMotor.setPower(0);
    }

}

