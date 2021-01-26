package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.utilities.PID;

/**
 * Created by Howard Paul on 10/30/2017.
 */

public class AlignwithGoldMineral extends BasicCommand {
    double heading,leftSpd,rightSpd,correction;
    PID goldPID;
    long timeOut;
    boolean centeredGold = false;
    long initTime = 0;

    public AlignwithGoldMineral(){
        this.heading = 0;
        this.leftSpd = .85;
        this.rightSpd = .85;
        this.correction = 0;
        goldPID = new PID(0.0028,0,0); // was 0.05

/*        if ((io.getAllianceColor() == IO.RED) && (io.getJewelColor() == IO.RED)) {
            headingPID.setTarget(heading_cw);
            heading = heading_cw;
        } else if ((io.getAllianceColor() == IO.RED) && (io.getJewelColor() == IO.BLUE)){
            headingPID.setTarget(heading_ccw);
            heading = heading_ccw;
        } else if ((io.getAllianceColor() == IO.BLUE) && (io.getJewelColor() == IO.BLUE)){
            headingPID.setTarget(heading_cw);
            heading = heading_cw;
        } else if ((io.getAllianceColor() == IO.BLUE) && (io.getJewelColor() == IO.RED)){
            headingPID.setTarget(heading_ccw);
            heading = heading_ccw;
        } else {
            headingPID.setTarget(heading);
        }*/
    }

    public void init() {
        timeOut = System.currentTimeMillis() + 9000;
        initTime = System.currentTimeMillis();
        goldPID.setTarget(0);
        /*if (io.isGoldFound) {
            goldPID.setTarget(0);
        }*/

        /*if ((io.getAllianceColor() == IO_4WD_Test.RED) && (io.getJewelColor() == IO_4WD_Test.RED)) {
            headingPID.setTarget(heading_cw);
            heading = heading_cw;
        } else if ((io.getAllianceColor() == IO_4WD_Test.RED) && (io.getJewelColor() == IO_4WD_Test.BLUE)){
            headingPID.setTarget(heading_ccw);
            heading = heading_ccw;
        } else if ((io.getAllianceColor() == IO_4WD_Test.BLUE) && (io.getJewelColor() == IO_4WD_Test.BLUE)){
            headingPID.setTarget(heading_cw);
            heading = heading_cw;
        } else if ((io.getAllianceColor() == IO_4WD_Test.BLUE) && (io.getJewelColor() == IO_4WD_Test.RED)){
            headingPID.setTarget(heading_ccw);
            heading = heading_ccw;
        } else {
            headingPID.setTarget(heading);
        }*/
    }

    public void execute(){

        if (io.getDOMPotDegrees() >= 35 && io.twoCyclesIsGoldFound && ((System.currentTimeMillis() - initTime) > 2500) && io.goldMineralFound ) {
            correction = goldPID.getCorrection(-io.getGoldXPositionAroundZero());
            correction = Range.clip(correction,-1,1);
            io.setDrivePower(correction*leftSpd,-correction*rightSpd);

            io.headingOfGold = Math.toDegrees(io.heading);
            if (Math.abs(io.headingOfGold) <= 10) {
                io.isGoldTheCenterMineral = true;
            } else {
                io.isGoldTheCenterMineral = false;
            }
            if (io.headingOfGold > 10) {
                io.isGoldTheRightMineral = true;
            } else {
                io.isGoldTheRightMineral = false;
            }
            if (io.headingOfGold < -10) {
                io.isGoldTheLeftMineral = true;
            } else {
                io.isGoldTheLeftMineral = false;
            }
        }

        if (io.getDOMPotDegrees() >= 35 && (Math.abs(io.getGoldXPositionAroundZero() - 0) <= 100) && io.twoCyclesIsGoldFound && io.twoCyclesIsGoldAligned && ((System.currentTimeMillis() - initTime) > 2500)) {
            centeredGold = true;
            io.isGoldCentered = true;
            io.headingOfGold = Math.toDegrees(io.heading);
            if (Math.abs(io.headingOfGold) <= 10) {
                io.isGoldTheCenterMineral = true;
            } else {
                io.isGoldTheCenterMineral = false;
            }
            if (io.headingOfGold > 10) {
                io.isGoldTheRightMineral = true;
            } else {
                io.isGoldTheRightMineral = false;
            }
            if (io.headingOfGold < -10) {
                io.isGoldTheLeftMineral = true;
            } else {
                io.isGoldTheLeftMineral = false;
            }

        } else {
            centeredGold = false;
            io.isGoldCentered = false;
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
        telemetry.addData("Mode:", "Align with Gold Mineral");
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
        telemetry.addData("Mode:", "Align with Gold Mineral");
        //return Math.abs(io.getHeading() - heading) <=2 || System.currentTimeMillis() >= timeOut;
        return (io.twoCyclesIsGoldCentered && io.goldMineralFound) || System.currentTimeMillis() >= timeOut;
        //return System.currentTimeMillis() >= timeOut;
    }
    public void stop() {
        io.setDrivePower(0,0);
    }

}
