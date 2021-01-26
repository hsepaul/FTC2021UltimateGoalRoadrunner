package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utilities.IO_4WD_Test;

import java.util.Locale;

/**
 * Created by HPaul on 10/25/2017.
 */

public class IdentifyProximity extends BasicCommand{

    long endTime;
    long foundAvgLeftProximity = 0;
    long foundAvgRightProximity = 0;
    int leftSampleCount = 0;
    int rightSampleCount = 0;
    double leftProximity = 0;
    double rightProximity = 0;
    double leftProximitySum = 0;
    double rightProximitySum = 0;
    double leftProximityAvg = 0;
    double rightProximityAvg = 0;

    public IdentifyProximity(){

    }

    public void init() {
        endTime = System.currentTimeMillis() + 5000;
        // send the info back to driver station using telemetry function.
        //telemetry.addData("Distance Left Sensor (cm)",
        //        String.format(Locale.US, "%.02f", io.leftDistance.getDistance(DistanceUnit.CM)));
        //telemetry.addData("Left Clear", io.leftColor.alpha());
        //telemetry.addData("Left Red  ", io.leftColor.red());
        //telemetry.addData("Left Green", io.leftColor.green());
        //telemetry.addData("Left Blue ", io.leftColor.blue());
        //telemetry.addData("Distance Right Sensor (cm)",
        //        String.format(Locale.US, "%.02f", io.rightDistance.getDistance(DistanceUnit.CM)));
        //telemetry.addData("Right Clear", io.rightColor.alpha());
        //telemetry.addData("Right Red  ", io.rightColor.red());
        //telemetry.addData("Right Green", io.rightColor.green());
        //telemetry.addData("Right Blue ", io.rightColor.blue());
        //telemetry.addData("Proximity Correction:", io.getProximityCorrection());
        //telemetry.addData("Left Proximity Average:", io.getLeftProximityAverage());
        //telemetry.addData("Right Proximity Average:", io.getRightProximityAverage());
        //telemetry.addData("Left Sample Count:", leftSampleCount);
        //telemetry.addData("Right Sample Count:", rightSampleCount);
        //telemetry.addData("Mode:", "Identify Proximity");
    }

    public void execute() {

        //leftProximity = io.leftDistance.getDistance(DistanceUnit.CM);
        if (!Double.isNaN(leftProximity) && leftSampleCount <= 20){
            leftSampleCount += 1;
            leftProximitySum += leftProximity;
            leftProximityAvg = leftProximitySum / leftSampleCount;
            foundAvgLeftProximity = System.currentTimeMillis();
        }

        //rightProximity = io.rightDistance.getDistance(DistanceUnit.CM);
        if (!Double.isNaN(rightProximity) && rightSampleCount <= 20){
            rightSampleCount += 1;
            rightProximitySum += rightProximity;
            rightProximityAvg = rightProximitySum / rightSampleCount;
            foundAvgRightProximity = System.currentTimeMillis();
        }


        telemetry.addData("Distance Left Sensor (cm)",
                String.format(Locale.US, "%.02f", leftProximity));
        //telemetry.addData("Left Clear", io.leftColor.alpha());
        //telemetry.addData("Left Red  ", io.leftColor.red());
        //telemetry.addData("Left Green", io.leftColor.green());
        //telemetry.addData("Left Blue ", io.leftColor.blue());
        telemetry.addData("Distance Right Sensor (cm)",
                String.format(Locale.US, "%.02f", rightProximity));
        //telemetry.addData("Right Clear", io.rightColor.alpha());
        //telemetry.addData("Right Red  ", io.rightColor.red());
        //telemetry.addData("Right Green", io.rightColor.green());
        //telemetry.addData("Right Blue ", io.rightColor.blue());
        //telemetry.addData("Proximity Correction:", io.getProximityCorrection());
        //telemetry.addData("Left Proximity Average:", io.getLeftProximityAverage());
        //telemetry.addData("Right Proximity Average:", io.getRightProximityAverage());
        telemetry.addData("Left Sample Count:", leftSampleCount);
        telemetry.addData("Right Sample Count:", rightSampleCount);
        telemetry.addData("Mode:", "Identify Proximity");
    }

    public boolean isFinished(){
        leftProximity = io.leftFrontDistance.getDistance(DistanceUnit.CM);
        rightProximity = io.rightFrontDistance.getDistance(DistanceUnit.CM);

        if (System.currentTimeMillis() >= endTime) {
            return true;
        }
        if (Double.isNaN(leftProximity) && Double.isNaN(rightProximity)) {
            //io.setLeftProximityAverage(0);
            //io.setRightProximityAverage(0);
            return false;
        } else if (Double.isNaN(leftProximity)) {
            //io.setLeftProximityAverage(0);
            //io.setRightProximityAverage(rightProximityAvg);
            return false;
        } else if (Double.isNaN(rightProximity)) {
            //io.setLeftProximityAverage(leftProximityAvg);
            //io.setRightProximityAverage(0);
            return false;
        } else if ((rightSampleCount < 20) && (leftSampleCount < 20)) {
            //io.setLeftProximityAverage(leftProximityAvg);
            //io.setRightProximityAverage(rightProximityAvg);
            return false;
        } else if ((rightSampleCount >= 20) || (leftSampleCount >= 20)) {
            //io.setLeftProximityAverage(leftProximityAvg);
            //io.setRightProximityAverage(rightProximityAvg);
            return true;
        } else {
            return false;
        }
    }
    public void stop() {
        io.setDrivePower(0,0, 0, 0);
    }
}
