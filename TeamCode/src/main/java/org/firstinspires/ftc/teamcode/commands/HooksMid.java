package org.firstinspires.ftc.teamcode.commands;


/**
 * Created by HPaul on 10/22/2017.
 */

public class HooksMid extends BasicCommand {
    long timeOut;
    long wakeupTime;

    boolean hooksMid = false;

    public HooksMid(long timeOut){ this.timeOut = timeOut; }

    public void init() {
        wakeupTime = System.currentTimeMillis() + timeOut;
        //timeOut = System.currentTimeMillis() + 3000;
        hooksMid = false;
    }

    public void execute(){
        telemetry.addData("Mode:", "Hooks Mid");
        io.rightHookMid();
        io.leftHookMid();
        hooksMid = true;
    }

    public boolean isFinished(){
        return hooksMid || System.currentTimeMillis() >= wakeupTime;
    }
    public void stop() {
        io.setDrivePower(0,0, 0, 0);
        //io.forkLiftMotor.setPower(0);
    }

}

