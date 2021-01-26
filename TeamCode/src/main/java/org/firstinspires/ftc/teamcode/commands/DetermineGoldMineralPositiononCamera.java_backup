package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.utilities.PID;

/**
 * Created by Howard Paul on 12/12/2018.
 */

public class DetermineGoldMineralPositiononCamera extends BasicCommand {
    long timeOut;
    boolean GoldMineralPositionCameraAverageComplete = false;
    double GoldMineralPositionCameraSum;
    long initTime = 0;
    int count = 0;

    public DetermineGoldMineralPositiononCamera(){
    }

    public void init() {
        timeOut = System.currentTimeMillis() + 9000;
        initTime = System.currentTimeMillis();
    }

    public void execute(){

        if (io.getDOMPotDegrees() >= 35 && io.twoCyclesIsGoldFound && (((System.currentTimeMillis() - initTime) > 1000) && ((System.currentTimeMillis() - initTime) <= 3000))) {

            GoldMineralPositionCameraSum += io.getGoldXPosition();
            count++;

            GoldMineralPositionCameraAverageComplete = false;
            io.GoldMineralPositionCameraAverageComplete = false;

        } else if ((System.currentTimeMillis() - initTime) <= 1000) {
            GoldMineralPositionCameraAverageComplete = false;
            io.GoldMineralPositionCameraAverageComplete = false;

        } else if ((System.currentTimeMillis() - initTime) > 3000) {

            // compute the average
            io.headingOfGold = 0;
            io.GoldMineralPositionCameraAverage = GoldMineralPositionCameraSum / count;

            if ((io.GoldMineralPositionCameraAverage >= 220) && (io.GoldMineralPositionCameraAverage <= 420)) {
                io.isGoldTheCenterMineral = true;
                io.headingOfGold = 0;
            } else {
                io.isGoldTheCenterMineral = false;
            }
            if ((io.GoldMineralPositionCameraAverage >= 439) && (io.GoldMineralPositionCameraAverage <= 639)) {
                io.isGoldTheRightMineral = true;
                io.headingOfGold = 20;
            } else {
                io.isGoldTheRightMineral = false;
            }
            if ((io.GoldMineralPositionCameraAverage >= 0) && (io.GoldMineralPositionCameraAverage <= 200)) {
                io.isGoldTheLeftMineral = true;
                io.headingOfGold = -20;
            } else {
                io.isGoldTheLeftMineral = false;
            }

            GoldMineralPositionCameraAverageComplete = true;
            io.GoldMineralPositionCameraAverageComplete = true;
        }

/*        telemetry.addData("Target Gold Position:", 0);
        //telemetry.addData("Heading: ", io.getHeading());
        telemetry.addData("Gold Position Around Zero: ", io.getGoldXPositionAroundZero());
        telemetry.addData("Heading: ", Math.toDegrees(io.heading));
        telemetry.addData("Completed Search: ", io.completedSearch );
        telemetry.addData("Captured Gold Heading: ", io.headingOfGold);
        telemetry.addData("Is Gold Center Mineral: ", io.isGoldTheCenterMineral);
        telemetry.addData("Two Cycles Gold Found: ", io.twoCyclesIsGoldFound);
        telemetry.addData("Two Cycles Gold Aligned: ", io.twoCyclesIsGoldAligned);
        telemetry.addData("Two Cycles Gold Centered: ", io.twoCyclesIsGoldCentered);
        telemetry.addData("Gold Position: ", io.getGoldXPosition());
        telemetry.addData("Is Gold Found: ", io.isGoldFound);
        telemetry.addData("Is Gold Aligned: ", io.isGoldAligned);
        telemetry.addData("Centered Gold: ", centeredGold);
        telemetry.addData("Correction: ", correction);
        telemetry.addData("Left Speed: ", leftSpd);
        telemetry.addData("Right Speed: ", rightSpd);
        //telemetry.addData("VuMark from IdentifyVuMark from IO", io.getVuMark());
        telemetry.addData("Potentiometer", String.format("%.01f degrees", (io.getDOMPotDegrees())));
        telemetry.addData("Mode:", "Align with Gold Mineral");*/
        telemetry.addData("Is Gold Left Mineral: ", io.isGoldTheLeftMineral);
        telemetry.addData("Is Gold Center Mineral: ", io.isGoldTheCenterMineral);
        telemetry.addData("Is Gold Right Mineral: ", io.isGoldTheRightMineral);
        telemetry.addData("Gold Mineral Position Camera Average: ", io.GoldMineralPositionCameraAverage);
        telemetry.addData("Gold Mineral Position Camera Counts: ", count);
        telemetry.addData("Gold Mineral Position Camera Average Complete: ", io.GoldMineralPositionCameraAverageComplete);
        telemetry.addData("Mode:", "Determine Gold Mineral Position On Camera");
    }

    public boolean isFinished(){
        /*telemetry.addData("Target Gold Position:", 0);
        //telemetry.addData("Heading: ", io.getHeading());
        telemetry.addData("Gold Position Around Zero: ", io.getGoldXPositionAroundZero());
        telemetry.addData("Heading: ", Math.toDegrees(io.heading));
        telemetry.addData("Completed Search: ", io.completedSearch );
        telemetry.addData("Captured Gold Heading: ", io.headingOfGold);
        telemetry.addData("Is Gold Center Mineral: ", io.isGoldTheCenterMineral);
        telemetry.addData("Two Cycles Gold Found: ", io.twoCyclesIsGoldFound);
        telemetry.addData("Two Cycles Gold Aligned: ", io.twoCyclesIsGoldAligned);
        telemetry.addData("Two Cycles Gold Centered: ", io.twoCyclesIsGoldCentered);
        telemetry.addData("Gold Position: ", io.getGoldXPosition());
        telemetry.addData("Is Gold Found: ", io.isGoldFound);
        telemetry.addData("Is Gold Aligned: ", io.isGoldAligned);
        telemetry.addData("Centered Gold: ", centeredGold);
        telemetry.addData("Correction: ", correction);
        telemetry.addData("Left Speed: ", leftSpd);
        telemetry.addData("Right Speed: ", rightSpd);
        //telemetry.addData("VuMark from IdentifyVuMark from IO", io.getVuMark());
        telemetry.addData("Potentiometer", String.format("%.01f degrees", (io.getDOMPotDegrees())));*/
        telemetry.addData("Mode:", "Determine Gold Mineral Position On Camera");
        //return Math.abs(io.getHeading() - heading) <=2 || System.currentTimeMillis() >= timeOut;
        telemetry.addData("Is Gold Left Mineral: ", io.isGoldTheLeftMineral);
        telemetry.addData("Is Gold Center Mineral: ", io.isGoldTheCenterMineral);
        telemetry.addData("Is Gold Right Mineral: ", io.isGoldTheRightMineral);
        telemetry.addData("Gold Mineral Position Camera Average: ", io.GoldMineralPositionCameraAverage);
        telemetry.addData("Gold Mineral Position Camera Counts: ", count);
        telemetry.addData("Gold Mineral Position Camera Average Complete: ", io.GoldMineralPositionCameraAverageComplete);
        return (GoldMineralPositionCameraAverageComplete) || System.currentTimeMillis() >= timeOut;
        //return System.currentTimeMillis() >= timeOut;
    }
    public void stop() {
        io.setDrivePower(0,0);
    }

}
