package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.utilities.PID;

/**
 * Created by David Austin on 10/27/2016.
 */

public class DriveSidewaysSkyStoneOpenCV extends BasicCommand {
    double targetPosition;
    double endDistance;
    double driveSpeed;
    PID distancePID;
    PID distanceForwardBack;
    PID headingPID;
    public static final int YGREATERTHAN = 0;
    public static final int XGREATERTHAN = 1;
    public static final int YLESSTHAN = 2;
    public static final int XLESSTHAN = 3;
    int test;
    //long endTime;
    long timeOut;
    long wakeupTime;
    double targetHeading;
    boolean coast = false;
    public DriveSidewaysSkyStoneOpenCV(double targetPosition, int test, double spd, double targetHeading, long timeOut){
        headingPID = new PID(0.02,0.00,0);
        //headingPID = new PID(0.02, 0.02, 0);
        //headingPID = new PID(0.05, 0, 0);
        headingPID.setTarget(targetHeading);
        distancePID = new PID(.3,0,0);
        distanceForwardBack = new PID(.3,0,0);
        //distancePID = new PID(.2,0,0);
        distancePID.setTarget(targetPosition);
        distanceForwardBack.setTarget(0);
        this.targetPosition = targetPosition;
        this.test = test;
        driveSpeed = spd;
        this.targetHeading = targetHeading;
        this.timeOut = timeOut;
    }
    public DriveSidewaysSkyStoneOpenCV(double targetPosition, int test, double spd, double targetHeading, long timeOut, boolean coast){
        this(targetPosition,test,spd,targetHeading, timeOut);
        this.coast=coast;
    }

    public DriveSidewaysSkyStoneOpenCV(double dist) {
        this(dist, YGREATERTHAN, 0.5, 0.0, 5000);
    }

    public void init(){
        wakeupTime = System.currentTimeMillis() + timeOut;
        //endTime = System.currentTimeMillis() + 30000;

        this.targetPosition = (targetPosition * io.skystoneDirection) + io.centerOffset;
        distancePID.setTarget(this.targetPosition);

        if (targetPosition > 0) {
            this.test = XGREATERTHAN;
            this.driveSpeed = this.driveSpeed;
        } else if (targetPosition < 0) {
            this.test = XLESSTHAN;
            this.driveSpeed = -this.driveSpeed;
        } else {
            this.test = 2;
        }

        //this.test = 2 - io.skystoneDirection;
        //this.driveSpeed = this.driveSpeed * io.skystoneDirection;

        /*if (usebutton){
            this.proximitybutton = io.proximityArmButtonPushed;
        }*/
    }

    public void execute(){
        //double heading = io.getHeading();
        double heading = Math.toDegrees(io.heading);
        double correction = headingPID.getCorrection(heading);
        double distanceCorrection;
        double distanceCorrectionForwardBack;
        switch(test) {
            case XGREATERTHAN:
            case XLESSTHAN:
                distanceCorrection = distancePID.getCorrection(io.getSidewaysDistance());
                distanceCorrectionForwardBack = distanceForwardBack.getCorrection(io.getX());
                break;
            default:
                distanceCorrection = distancePID.getCorrection(io.getSidewaysDistance());
                distanceCorrectionForwardBack = distanceForwardBack.getCorrection(io.getX());
                break;
        }




        distanceCorrection = Range.clip(Math.abs(distanceCorrection),0,1);
        correction = Range.clip(correction,-1,1);
        distanceCorrectionForwardBack = Range.clip(distanceCorrectionForwardBack,-1,1);
        distanceCorrectionForwardBack = 0; //remove forward/back correction PID
        double frontleftSpeed = (-driveSpeed * distanceCorrection) - correction + distanceCorrectionForwardBack;
        double frontrightSpeed = (driveSpeed * distanceCorrection) + correction + distanceCorrectionForwardBack;
        double backleftSpeed = (driveSpeed * distanceCorrection) - correction + distanceCorrectionForwardBack;
        double backrightSpeed = (-driveSpeed * distanceCorrection) + correction + distanceCorrectionForwardBack;
        if (driveSpeed > 0) {
            frontleftSpeed = Range.clip(frontleftSpeed, -1, 0);
            frontrightSpeed = Range.clip(frontrightSpeed, 0, 1);
            backleftSpeed = Range.clip(backleftSpeed, 0, 1);
            backrightSpeed = Range.clip(backrightSpeed, -1, 0);
        } else {
            frontleftSpeed = Range.clip(frontleftSpeed, 0, 1);
            frontrightSpeed = Range.clip(frontrightSpeed, -1, 0);
            backleftSpeed = Range.clip(backleftSpeed, -1, 0);
            backrightSpeed = Range.clip(backrightSpeed, 0, 1);
        }

        io.setDrivePower(frontleftSpeed,frontrightSpeed,backleftSpeed,backrightSpeed);
        //telemetry.addData("x: ",io.getX());
        //telemetry.addData("y: ",io.getY());
        /*telemetry.addData("sideways: ",io.getSidewaysDistance());
        telemetry.addData("skystone1Distance: ",io.skystone1Distance);
        telemetry.addData("skystoneOffsetDistanceToTape: ",io.skystoneOffsetDistanceToTape);
        telemetry.addData("skystone2Distance: ",io.skystone2Distance);*/

        /*telemetry.addData("center after reset get sideways distance: ",io.getSidewaysDistance());
        telemetry.addData("center after reset get odometer center encoder: ",io.getOdometerCenterEncoder());
        telemetry.addData("center after reset back right motor encoder: ",io.backRightMotor.getCurrentPosition());

        telemetry.addData("right after reset get x distance: ",io.getX());
        telemetry.addData("right after reset get odometer right encoder: ",io.getOdometerRightEncoder());
        telemetry.addData("right after reset back right motor encoder: ",io.frontLeftMotor.getCurrentPosition());

        telemetry.addData("left after reset get x distance: ",io.getX());
        telemetry.addData("left after reset get odometer right encoder: ",io.getOdometerLeftEncoder());
        telemetry.addData("left after reset back right motor encoder: ",io.frontRightMotor.getCurrentPosition());*/


        /*telemetry.addData("Target Heading:", targetHeading);
        telemetry.addData("Heading:", heading);
        telemetry.addData("Heading Correction: ", correction);
        telemetry.addData("Distance Correction: ", distanceCorrection);
        telemetry.addData("Drive Speed: ", driveSpeed);
        telemetry.addData("Front Left Speed: ", frontleftSpeed);
        telemetry.addData("Front Right Speed: ", frontrightSpeed);
        telemetry.addData("Back Left Speed: ", backleftSpeed);
        telemetry.addData("Back Right Speed: ", backrightSpeed);*/
        //telemetry.addData("Jewel Color is Unknown, Red, Blue: ", io.getJewelColor());
        //telemetry.addData("Alliance Color is Unknown, Red, Blue: ", io.getAllianceColor());
        //telemetry.addData("VuMark from IdentifyVuMark from IO", io.getVuMark());
        //telemetry.addData("VuMark from IdentifyVuMark from IO", "%s visible", io.vuMark);
        telemetry.addData("Mode:", "Drive Sideways");
    }

    public boolean isFinished(){
        if (System.currentTimeMillis() >= wakeupTime) return true;
        /*if ((io.touchProximity.getState() == false) && (usebutton == false)){
            io.proximityArmButtonPushed = true;
            return true;
        }*/
        //if ((usebutton == true) && (proximitybutton == false)) return true;
        //telemetry.addData("x: ",io.getX());
        //telemetry.addData("y: ",io.getY());
        //telemetry.addData("Target Heading:", targetHeading);
        //telemetry.addData("Heading:", io.getHeading());
        //telemetry.addData("Heading Correction: ", headingPID.getCorrection(io.getHeading()));
        //telemetry.addData("Distance Correction: ", distancePID.getCorrection(io.getY()));
        //telemetry.addData("Drive Speed: ", driveSpeed);
        //telemetry.addData("Left Speed: ", (driveSpeed * distancePID.getCorrection(io.getY())) - headingPID.getCorrection(io.getHeading()));
        //telemetry.addData("Right Speed: ", (driveSpeed * distancePID.getCorrection(io.getY())) + headingPID.getCorrection(io.getHeading()));
        switch(test) {
            case XGREATERTHAN:
                return io.getSidewaysDistance() >= targetPosition;
            case XLESSTHAN:
                return io.getSidewaysDistance() <= targetPosition;
            default:
                return true;
        }
    }

    public void stop(){
        if (!coast) io.setDrivePower(0.0,0.0, 0.0, 0.0);
    }

}
