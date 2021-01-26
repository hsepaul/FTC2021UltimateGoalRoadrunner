package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utilities.PID;

/**
 * Created by Howard Paul on 11/8/2018.
 */

public class DriveForwardHeadingandDistanceSensorSkyStone extends BasicCommand {
    double targetPosition;
    double endDistance;
    double driveSpeed;
    double heading;
    double leftSpeed;
    double rightSpeed;
    double correction;
    double distanceCorrection;
    PID headingPID;
    PID distancePID;
    public static final int LEFTGREATERTHAN = 0;
    public static final int RIGHTGREATERTHAN = 1;
    public static final int LEFTLESSTHAN = 2;
    public static final int RIGHTLESSTHAN = 3;
    int test;
    //long endTime;
    long timeOut;
    long wakeupTime;
    double targetHeading;
    boolean coast = false;
    public DriveForwardHeadingandDistanceSensorSkyStone(double targetPosition, int test, double spd, double targetHeading, long timeOut){
        headingPID = new PID(0.02,0,0);
        headingPID.setTarget(targetHeading);
        distancePID = new PID(0.2,0,0);
        distancePID.setTarget(targetPosition);
        this.targetPosition = targetPosition;
        this.test = test;
        driveSpeed = spd;
        this.targetHeading = targetHeading;
        this.timeOut = timeOut;
    }

    public DriveForwardHeadingandDistanceSensorSkyStone(double targetPosition, int test, double spd, double targetHeading, long timeOut, boolean coast){
        this(targetPosition,test,spd,targetHeading, timeOut);
        this.coast=coast;
    }

    public void init(){
        wakeupTime = System.currentTimeMillis() + timeOut;
        //endTime = System.currentTimeMillis() + 5000;
    }

    public void execute(){

        heading = Math.toDegrees(io.heading);
        correction = headingPID.getCorrection(heading);

        switch(test) {
            case LEFTGREATERTHAN:
            case LEFTLESSTHAN:
                distanceCorrection = distancePID.getCorrection(io.leftFrontDistance.getDistance(DistanceUnit.INCH));
                break;
            case RIGHTGREATERTHAN:
            case RIGHTLESSTHAN:
            default:
                distanceCorrection = distancePID.getCorrection(io.rightFrontDistance.getDistance(DistanceUnit.INCH));
                break;
        }
        distanceCorrection = Range.clip(Math.abs(distanceCorrection),0,1);
        correction = Range.clip(correction,-1,1);
        leftSpeed = (-driveSpeed * distanceCorrection) - correction;
        rightSpeed = (-driveSpeed * distanceCorrection) + correction;
        if (driveSpeed > 0) {
            leftSpeed = Range.clip(leftSpeed, -1, 0);
            rightSpeed = Range.clip(rightSpeed, -1, 0);
        } else {
            leftSpeed = Range.clip(leftSpeed, 0, 1);
            rightSpeed = Range.clip(rightSpeed, 0, 1);
        }

        io.setDrivePower(leftSpeed,rightSpeed,leftSpeed,rightSpeed);
        /*telemetry.addData("Front Distance: ",io.leftFrontDistance.getDistance(DistanceUnit.INCH));
        //telemetry.addData("Back Distance: ",io.backDistance.getDistance(DistanceUnit.INCH));
        telemetry.addData("Target Heading:", targetHeading);
        telemetry.addData("Heading:", heading);
        telemetry.addData("Heading Correction: ", correction);
        telemetry.addData("Distance Correction: ", distanceCorrection);
        telemetry.addData("Drive Speed: ", driveSpeed);
        telemetry.addData("Left Speed: ", leftSpeed);
        telemetry.addData("Right Speed: ", rightSpeed);
        telemetry.addData("Alliance Color is Unknown, Red, Blue: ", io.getAllianceColor());*/
        telemetry.addData("Mode:", "Drive Forward Heading and Distance Sensor");
    }

    public boolean isFinished(){
        if (System.currentTimeMillis() >= wakeupTime) return true;
        /*telemetry.addData("Front Distance: ",io.leftFrontDistance.getDistance(DistanceUnit.INCH));
        //telemetry.addData("Back Distance: ",io.backDistance.getDistance(DistanceUnit.INCH));
        telemetry.addData("Target Heading:", targetHeading);
        telemetry.addData("Heading:", heading);
        telemetry.addData("Heading Correction: ", correction);
        telemetry.addData("Distance Correction: ", distanceCorrection);
        telemetry.addData("Drive Speed: ", driveSpeed);
        telemetry.addData("Left Speed: ", leftSpeed);
        telemetry.addData("Right Speed: ", rightSpeed);
        telemetry.addData("Alliance Color is Unknown, Red, Blue: ", io.getAllianceColor());*/
        telemetry.addData("Mode:", "Drive Forward Heading and Distance Sensor");

        switch(test) {
            case LEFTGREATERTHAN:
                return io.leftFrontDistance.getDistance(DistanceUnit.INCH) > targetPosition;
            case LEFTLESSTHAN:
                return io.leftFrontDistance.getDistance(DistanceUnit.INCH) < targetPosition;
            case RIGHTGREATERTHAN:
                return io.rightFrontDistance.getDistance(DistanceUnit.INCH) > targetPosition;
            case RIGHTLESSTHAN:
            default:
                return io.rightFrontDistance.getDistance(DistanceUnit.INCH) < targetPosition;
        }
    }

    public void stop(){
        if (!coast) io.setDrivePower(0.0,0.0, 0.0, 0.0);
    }

}
