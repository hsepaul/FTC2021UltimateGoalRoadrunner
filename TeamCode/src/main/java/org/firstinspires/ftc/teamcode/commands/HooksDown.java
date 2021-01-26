package org.firstinspires.ftc.teamcode.commands;


/**
 * Created by HPaul on 10/22/2017.
 */

public class HooksDown extends BasicCommand {
    long timeOut;
    long wakeupTime;

    boolean hooksDown = false;

    public HooksDown(long timeOut){ this.timeOut = timeOut; }

    public void init() {
        wakeupTime = System.currentTimeMillis() + timeOut;
        //timeOut = System.currentTimeMillis() + 3000;
        hooksDown = false;
    }

    public void execute(){
        telemetry.addData("Mode:", "Hooks Down");
        io.rightHookDown();
        io.leftHookDown();
        hooksDown = true;
    }

    public boolean isFinished(){
        return hooksDown || System.currentTimeMillis() >= wakeupTime;
    }
    public void stop() {
        io.setDrivePower(0,0, 0, 0);
        //io.forkLiftMotor.setPower(0);
    }

}

