package org.firstinspires.ftc.teamcode.commands;


/**
 * Created by HPaul on 10/22/2017.
 */

public class HooksUp extends BasicCommand {
    long timeOut;
    long wakeupTime;

    boolean hooksUp = false;

    public HooksUp(long timeOut){ this.timeOut = timeOut; }

    public void init() {
        wakeupTime = System.currentTimeMillis() + timeOut;
        //timeOut = System.currentTimeMillis() + 3000;
        hooksUp = false;
    }

    public void execute(){
        telemetry.addData("Mode:", "Hooks Up");
        io.rightHookUp();
        io.leftHookUp();
        hooksUp = true;
    }

    public boolean isFinished(){
        return hooksUp || System.currentTimeMillis() >= wakeupTime;
    }
    public void stop() {
        io.setDrivePower(0,0, 0, 0);
        //io.forkLiftMotor.setPower(0);
    }

}

