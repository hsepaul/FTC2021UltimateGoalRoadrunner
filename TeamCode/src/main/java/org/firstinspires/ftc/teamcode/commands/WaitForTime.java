package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;


/**
 * Created by HPaul on 10/22/2017.
 */

public class WaitForTime extends BasicCommand {
    long timeOut;
    long wakeupTime;
    long initTime;

    public  WaitForTime (long timeOut){
            this.timeOut = timeOut;
        }

    public void init() {
        initTime = System.currentTimeMillis();
        wakeupTime = initTime + timeOut;
        //timeOut = System.currentTimeMillis() + wakeupTime;
    }

    public void execute(){
        telemetry.addData("Mode:", "WaitForTime");
        telemetry.addData("Time To Wait (ms)", timeOut);
        telemetry.addData("Time Waited (ms)", System.currentTimeMillis() - initTime);
        telemetry.addData("x after reset: ",io.getX());
        telemetry.addData("y after reset: ",io.getY());
        telemetry.addData("IMU Heading:", io.angles.firstAngle);
        telemetry.addData("IMU IO Heading: ", Math.toDegrees(io.heading));

        /*telemetry.addData("Distance left sensor (cm)",
                String.format(Locale.US, "%.02f", io.leftDistance.getDistance(DistanceUnit.CM)));

        telemetry.addData("Distance right sensor(cm)",
                String.format(Locale.US, "%.02f", io.rightDistance.getDistance(DistanceUnit.CM)));*/
        //telemetry.addData("Proximity Correction:", io.getProximityCorrection());
        //telemetry.addData("Left Proximity Average:", io.getLeftProximityAverage());
        //telemetry.addData("Right Proximity Average:", io.getRightProximityAverage());
        //telemetry.addData("VuMark from IdentifyVuMark from IO", io.getVuMark());
    }

    public boolean isFinished(){
            return System.currentTimeMillis() >= wakeupTime;
        }
    public void stop() {
            io.setDrivePower(0,0, 0, 0);
        }

}

