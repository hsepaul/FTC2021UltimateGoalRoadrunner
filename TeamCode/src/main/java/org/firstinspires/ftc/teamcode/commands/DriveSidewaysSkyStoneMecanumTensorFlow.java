package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.internal.vuforia.VuforiaLocalizerImpl;
import org.firstinspires.ftc.teamcode.utilities.PID;

import java.util.List;

/**
 * Created by David Austin on 10/27/2016.
 */

public class DriveSidewaysSkyStoneMecanumTensorFlow extends BasicCommand {
    double targetPosition;
    double endDistance;
    double driveSpeed;
    PID distancePID;
    PID distanceForwardBack;
    PID headingPID;
    //long endTime;
    long timeOut;
    long wakeupTime;
    double targetHeading;
    boolean coast = false;

    double angleSkystone = 100;

    WebcamName webcamName;

    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    /*private static final String VUFORIA_KEY =
            "Ae+uGTX/////AAABmQV3De8djUCDjn2zDZDbCssvJv8/irA8Dzm+UnPYeGcgN7Y/V1EFU/DgmBcA3x5TxqeooD4B02M6PR+5IBifNlYVIXezFdgl/f9PKHDE7KAl3yeEV993njRk8ocjpNJwYDqcN1vZP6yWRqe4Y9QdAJH+KZPQeR+eN5wT87m4ZNHhsC5DidIkFYuhVNVdM+Gn9CLUphmjX1woXqSLqK3BdmU6XEfKU730USi7clKwVidBUMCcFcL878gUG0Mn5JL7dcPUO3r1q+8ODt1wInwPWgSQlXrrY4wWSeHJ5VwwihGnisIZ2Ps41yqf1QtrzK7FsDz5P5aQaQ7rVtzntFLZZ+ftIy0aJ+YelBy1QtZX+dc8";
*//*    private static VuforiaLocalizer vuforia;
    private static VuforiaLocalizer.Parameters parameters;*/
    private TFObjectDetector tfod;

    public DriveSidewaysSkyStoneMecanumTensorFlow(double targetPosition, double spd, double targetHeading, long timeOut){
        headingPID = new PID(0.03,0.00,0);
        //headingPID = new PID(0.02, 0.02, 0);
        //headingPID = new PID(0.05, 0, 0);
        headingPID.setTarget(targetHeading);
        distancePID = new PID(.4,0,0);
        distanceForwardBack = new PID(.4,0,0);
        //distancePID = new PID(.2,0,0);
        distancePID.setTarget(targetPosition);
        distanceForwardBack.setTarget(0);
        this.targetPosition = targetPosition;
        //this.test = test;
        driveSpeed = spd;
        this.targetHeading = targetHeading;
        this.timeOut = timeOut;
    }
    public DriveSidewaysSkyStoneMecanumTensorFlow(double targetPosition, double spd, double targetHeading, long timeOut, boolean coast){
        this(targetPosition,spd,targetHeading, timeOut);
        this.coast=coast;
    }

    public DriveSidewaysSkyStoneMecanumTensorFlow(double dist) {
        this(dist, 0.5, 0.0, 5000);
    }

    public void init() {

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.

        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */

        /*webcamName = io.webcamName;

        if(parameters==null) {
            parameters = new VuforiaLocalizer.Parameters();

            parameters.vuforiaLicenseKey = VUFORIA_KEY;
            parameters.cameraName = webcamName;
        }
        //parameters.cameraName = map.get(WebcamName.class, "Webcam 1");

        //telemetry.addData("webcamName.isAttached(): ",webcamName.isAttached());
        //telemetry.addData("vuforia==null: ",vuforia==null);

        //  Instantiate the Vuforia engine
        if (vuforia == null) {
        //if (!webcamName.isAttached()) {
            vuforia = ClassFactory.getInstance().createVuforia(parameters);
        }
        //}*/


        // Loading trackables is not necessary for the TensorFlow Object Detection engine.

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            int tfodMonitorViewId = map.appContext.getResources().getIdentifier(
                    "tfodMonitorViewId", "id", map.appContext.getPackageName());
            TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
            tfodParameters.minimumConfidence = 0.6;
            tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, io.vuforia);

            // Try to only load Skystone
            //tfod.loadModelFromAsset(TFOD_MODEL_ASSET, null, LABEL_SECOND_ELEMENT);
            tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
        }

        wakeupTime = System.currentTimeMillis() + timeOut;
        //endTime = System.currentTimeMillis() + 12000;

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

        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    if (recognition.getLabel() == "Skystone") {
                        angleSkystone = recognition.estimateAngleToObject(AngleUnit.DEGREES);
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());
                        telemetry.addData(String.format("  estimated angle (%d)", i), "%.03f",
                                recognition.estimateAngleToObject(AngleUnit.DEGREES));
                        break;
                    }
                }
                telemetry.update();
            } else {
                angleSkystone = 100;
            }
        }

        distanceCorrection = distancePID.getCorrection(angleSkystone);
        distanceCorrectionForwardBack = distanceForwardBack.getCorrection(io.getX());


        /*switch(test) {
            case XGREATERTHAN:
            case XLESSTHAN:
                distanceCorrection = distancePID.getCorrection(angleSkystone);
                distanceCorrectionForwardBack = distanceForwardBack.getCorrection(io.getX());
                break;
            default:
                distanceCorrection = distancePID.getCorrection(angleSkystone);
                distanceCorrectionForwardBack = distanceForwardBack.getCorrection(io.getX());
                break;
        }*/

/*#REMOVE
        double cosA = Math.cos(Math.toRadians(0.0));
        double sinA = Math.sin(Math.toRadians(0.0));
        double x1 = -gamepad1.left_stick_x*cosA - gamepad1.left_stick_y*sinA;
        double y1 = -gamepad1.left_stick_x*sinA + gamepad1.left_stick_y*cosA;
        double rotation = -gamepad1.right_stick_x;


        double[] wheelPowers = new double[4];
        wheelPowers[0] = x1 + y1 + rotation;   //FL
        wheelPowers[1] = -x1 + y1 - rotation;  //FR
        wheelPowers[2] = -x1 + y1 + rotation;  //BL
        wheelPowers[3] = x1 + y1 - rotation;   //BR
        // #REMOVE*/




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
        telemetry.addData("Target Heading:", targetHeading);
        telemetry.addData("Heading:", heading);
        telemetry.addData("Heading Correction: ", correction);
        telemetry.addData("Distance Correction: ", distanceCorrection);
        telemetry.addData("Drive Speed: ", driveSpeed);
        telemetry.addData("Front Left Speed: ", frontleftSpeed);
        telemetry.addData("Front Right Speed: ", frontrightSpeed);
        telemetry.addData("Back Left Speed: ", backleftSpeed);
        telemetry.addData("Back Right Speed: ", backrightSpeed);
        //telemetry.addData("Jewel Color is Unknown, Red, Blue: ", io.getJewelColor());
        telemetry.addData("Alliance Color is Unknown, Red, Blue: ", io.getAllianceColor());
        //telemetry.addData("VuMark from IdentifyVuMark from IO", io.getVuMark());
        //telemetry.addData("VuMark from IdentifyVuMark from IO", "%s visible", io.vuMark);*/
        telemetry.addData("Mode:", "Drive Sideways");
    }

    public boolean isFinished(){
        //if (System.currentTimeMillis() >= endTime) return true;
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

        if ((Math.abs(angleSkystone - targetPosition) <= 4) && !io.skystone1Found && !io.skystone2Found) {
            io.skystone1Found = true;
            io.skystone2Found = false;
            io.skystone1Distance = io.getSidewaysDistance();
        } else if ((Math.abs(angleSkystone - targetPosition) <= 4) && io.skystone1Found && !io.skystone2Found){
            io.skystone1Found = true;
            io.skystone2Found = true;
            io.skystone2Distance = io.getSidewaysDistance();
        }

        /*telemetry.addData("skystone1Found: ",io.skystone1Found);
        telemetry.addData("skystone1Distance: ",io.skystone1Distance);
        telemetry.addData("skystone2Found: ",io.skystone2Found);
        telemetry.addData("skystone2Distance: ",io.skystone2Distance);
        telemetry.addData("angleSkystone: ",angleSkystone);
        telemetry.addData("targetPosition: ",targetPosition);*/

        return Math.abs(angleSkystone - targetPosition) <= 4 || System.currentTimeMillis() >= wakeupTime;
        /*switch(test) {
            case XGREATERTHAN:
                return io.getSidewaysDistance() > targetPosition;
            case XLESSTHAN:
                return io.getSidewaysDistance() < targetPosition;
            default:
                return io.getSidewaysDistance() < targetPosition;
        }*/
    }

    public void stop(){
        if (tfod != null) {
            tfod.shutdown();
        }

        //vuforia.getCamera().close();
        //webcamName.close();
        //io.webcamName.close();
        if (!coast) io.setDrivePower(0.0,0.0, 0.0, 0.0);
    }

}
