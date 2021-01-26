package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.utilities.PID;
import org.firstinspires.ftc.teamcode.utilities.HSPID;
import org.firstinspires.ftc.teamcode.utilities.IO_4WD_Test;

/**
 * Created by Howard Paul on 10/30/2017.
 */

public class RemoveOpponentsJewel extends BasicCommand {
    double heading,heading_cw,heading_ccw,leftSpd,rightSpd;
    PID headingPID;
    long timeOut;

    public RemoveOpponentsJewel(){
        this.heading_cw = 15; //clockwise 25 degrees
        this.heading_ccw = -15; //counterclockwise 25 degrees
        this.heading = 0;
        this.leftSpd = .7;
        this.rightSpd = .7;
        headingPID = new PID(0.065,0,0); // was 0.05

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
        timeOut = System.currentTimeMillis() + 6000;

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
        io.setDrivePower(correction*leftSpd,-correction*rightSpd, correction*leftSpd,-correction*rightSpd);
        telemetry.addData("Target Heading:", heading);
        //telemetry.addData("Heading: ", io.getHeading());
        telemetry.addData("Heading: ", Math.toDegrees(io.heading));
        telemetry.addData("Correction: ", correction);
        telemetry.addData("Left Speed: ", leftSpd);
        telemetry.addData("Right Speed: ", rightSpd);
        //telemetry.addData("VuMark from IdentifyVuMark from IO", io.getVuMark());
        telemetry.addData("Mode:", "Rotate");
    }

    public boolean isFinished(){
        telemetry.addData("Target Heading:", heading);
        //telemetry.addData("Heading: ", io.getHeading());
        telemetry.addData("Heading: ", Math.toDegrees(io.heading));
        telemetry.addData("x: ",io.getX());
        telemetry.addData("y: ",io.getY());
        //telemetry.addData("Correction: ", headingPID.getCorrection(io.getHeading()));
        telemetry.addData("Correction: ", headingPID.getCorrection(Math.toDegrees(io.heading)));
        telemetry.addData("Left Speed: ", leftSpd);
        telemetry.addData("Right Speed: ", rightSpd);
        //telemetry.addData("VuMark from IdentifyVuMark from IO", io.getVuMark());
        //return Math.abs(io.getHeading() - heading) <=2 || System.currentTimeMillis() >= timeOut;
        return Math.abs(Math.toDegrees(io.heading) - heading) <=2.3 || System.currentTimeMillis() >= timeOut;
    }
    public void stop() {
        io.setDrivePower(0,0, 0, 0);
    }

}
