package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.utilities.PID;

/**
 * Created by Howard Paul on 10/30/2017.
 */

public class FindGoldMineral extends BasicCommand {
    double heading,heading_cw,heading_ccw,leftSpd,rightSpd;
    PID headingPID;
    long timeOut;
    long initTime = 0;
    boolean rotatedCW = false;
    boolean rotatedCCW = false;
    boolean completedSearch = false;

    public FindGoldMineral(){
        this.heading_cw = 12; //clockwise 10 degrees
        this.heading_ccw = -12; //counterclockwise 10 degrees
        this.heading = 0;
        this.leftSpd = 1;
        this.rightSpd = 1;
        headingPID = new PID(0.05,0,0); // was 0.05

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
        timeOut = System.currentTimeMillis() + 5000;
        initTime = System.currentTimeMillis();

        headingPID.setTarget(heading_cw);
        heading = heading_cw;

        /*if (!io.isGoldFound) {
            headingPID.setTarget(heading_cw);
            heading = heading_cw;
        }else {
            headingPID.setTarget(heading);
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
        //double correction = headingPID.getCorrection(io.getHeading());
        double correction = headingPID.getCorrection(Math.toDegrees(io.heading));
        correction = Range.clip(correction,-1,1);
        //correction = Range.clip(correction,0,1);
        if (io.getDOMPotDegrees() >= 35 && ((System.currentTimeMillis() - initTime) > 1000)) {
            io.setDrivePower(correction*leftSpd,-correction*rightSpd);
        }

        if ((Math.abs(Math.toDegrees(io.heading) - heading) <= 2) && !rotatedCW) {
            rotatedCW = true;
            headingPID.setTarget(heading_ccw);
            heading = heading_ccw;
        }

        if ((rotatedCW) && (Math.abs(Math.toDegrees(io.heading) - heading) <= 2)) {
            rotatedCCW = true;
            completedSearch = true;
            io.completedSearch = true;
        }


        /*telemetry.addData("Target Heading:", heading);
        //telemetry.addData("Heading: ", io.getHeading());
        telemetry.addData("Heading: ", Math.toDegrees(io.heading));
        telemetry.addData("Completed Search: ", io.completedSearch );
        telemetry.addData("Correction: ", correction);
        telemetry.addData("Left Speed: ", leftSpd);
        telemetry.addData("Right Speed: ", rightSpd);
        //telemetry.addData("VuMark from IdentifyVuMark from IO", io.getVuMark());
        telemetry.addData("Potentiometer", String.format("%.01f degrees", (io.getDOMPotDegrees())));*/
        telemetry.addData("Mode:", "Find Gold Mineral");
    }

    public boolean isFinished(){
        /*telemetry.addData("Target Heading:", heading);
        //telemetry.addData("Heading: ", io.getHeading());
        telemetry.addData("Heading: ", Math.toDegrees(io.heading));
        telemetry.addData("Completed Search: ", io.completedSearch );
        telemetry.addData("x: ",io.getX());
        telemetry.addData("y: ",io.getY());
        //telemetry.addData("Correction: ", headingPID.getCorrection(io.getHeading()));
        telemetry.addData("Correction: ", headingPID.getCorrection(Math.toDegrees(io.heading)));
        telemetry.addData("Left Speed: ", leftSpd);
        telemetry.addData("Right Speed: ", rightSpd);*/
        //telemetry.addData("VuMark from IdentifyVuMark from IO", io.getVuMark());
        //return Math.abs(io.getHeading() - heading) <=2 || System.currentTimeMillis() >= timeOut;
        telemetry.addData("Mode:", "Find Gold Mineral");
        if (((io.twoCyclesIsGoldFound || completedSearch || System.currentTimeMillis() >= timeOut) && ((System.currentTimeMillis() - initTime) > 1000))) {
            io.goldMineralFound = true;
        } else {
            io.goldMineralFound = false;
        }
        return ((io.twoCyclesIsGoldFound || completedSearch || System.currentTimeMillis() >= timeOut) && ((System.currentTimeMillis() - initTime) > 1000));
    }
    public void stop() {
        io.setDrivePower(0,0);
    }

}
