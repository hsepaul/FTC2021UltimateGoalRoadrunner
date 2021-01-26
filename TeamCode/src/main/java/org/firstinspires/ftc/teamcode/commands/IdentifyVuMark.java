package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuforiaNavigation;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.utilities.IO_4WD_Test;

/**
 * Created by HPaul on 10/25/2017.
 */

public class IdentifyVuMark extends BasicCommand{

    VuforiaLocalizer vuforia;
    VuforiaTrackable relicTemplate;
    RelicRecoveryVuMark vuMark;

    long timeOut;

    public  IdentifyVuMark (){

    }

    public void init() {
        /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        timeOut = System.currentTimeMillis() + 9000;
        int cameraMonitorViewId = io.cameraMonitorViewId;
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        /*
         * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
         * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
         * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
         * web site at https://developer.vuforia.com/license-manager.
         *
         * Vuforia license keys are always 380 characters long, and look as if they contain mostly
         * random data. As an example, here is a example of a fragment of a valid key:
         *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
         * Once you've obtained a license key, copy the string from the Vuforia web site
         * and paste it in to your code onthe next line, between the double quotes.
         */
        parameters.vuforiaLicenseKey = "Ae+uGTX/////AAABmQV3De8djUCDjn2zDZDbCssvJv8/irA8Dzm+UnPYeGcgN7Y/V1EFU/DgmBcA3x5TxqeooD4B02M6PR+5IBifNlYVIXezFdgl/f9PKHDE7KAl3yeEV993njRk8ocjpNJwYDqcN1vZP6yWRqe4Y9QdAJH+KZPQeR+eN5wT87m4ZNHhsC5DidIkFYuhVNVdM+Gn9CLUphmjX1woXqSLqK3BdmU6XEfKU730USi7clKwVidBUMCcFcL878gUG0Mn5JL7dcPUO3r1q+8ODt1wInwPWgSQlXrrY4wWSeHJ5VwwihGnisIZ2Ps41yqf1QtrzK7FsDz5P5aQaQ7rVtzntFLZZ+ftIy0aJ+YelBy1QtZX+dc8";

        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
        relicTrackables.activate();
    }

    public void execute(){
        /**
         * See if any of the instances of {@link relicTemplate} are currently visible.
         * {@link RelicRecoveryVuMark} is an enum which can have the following values:
         * UNKNOWN, LEFT, CENTER, and RIGHT. When a VuMark is visible, something other than
         * UNKNOWN will be returned by {@link RelicRecoveryVuMark#from(VuforiaTrackable)}.
         */
        vuMark = RelicRecoveryVuMark.from(relicTemplate);
        if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
            telemetry.addData("VuMark", "%s visible", vuMark);

                /* For fun, we also exhibit the navigational pose. In the Relic Recovery game,
                 * it is perhaps unlikely that you will actually need to act on this pose information, but
                 * we illustrate it nevertheless, for completeness. */
            OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();
            telemetry.addData("Pose", format(pose));

                /* We further illustrate how to decompose the pose into useful rotational and
                 * translational components */
            if (pose != null) {
                VectorF trans = pose.getTranslation();
                Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                // Extract the X, Y, and Z components of the offset of the target relative to the robot
                double tX = trans.get(0);
                double tY = trans.get(1);
                double tZ = trans.get(2);

                // Extract the rotational components of the target relative to the robot
                double rX = rot.firstAngle;
                double rY = rot.secondAngle;
                double rZ = rot.thirdAngle;
            }
        }
        else {
            telemetry.addData("VuMark", "not visible");
        }

        telemetry.addData("Mode:", "Identify VuMark");
    }

    public boolean isFinished(){
        if (vuMark == null) {
            //io.setVuMark(IO_4WD_Test.UNKNOWN);
            return false;
        } else {
            //io.setVuMark(vuMark.ordinal());
            //telemetry.addData("VuMark from IdentifyVuMark from IO", io.getVuMark());
            telemetry.addData("VuMark from IdentifyVuMark from IO", "%s visible", vuMark);
            /*if (vuMark == RelicRecoveryVuMark.LEFT ) {
                io.setVuMark(IO_4WD_Test.LEFT);
                return true;
            } else if (vuMark == RelicRecoveryVuMark.CENTER){
                io.setVuMark(IO_4WD_Test.CENTER);
                return true;
            } else if (vuMark == RelicRecoveryVuMark.RIGHT) {
                io.setVuMark(IO_4WD_Test.RIGHT);
                return true;
            } else {
                telemetry.addData("vuMark", "Unknown");
                telemetry.addData("vuMark", "not identified");
                //io.setVuMark(IO_4WD_Test.UNKNOWN);
                return false;
            }*/
            //return vuMark == (RelicRecoveryVuMark.LEFT || RelicRecoveryVuMark.RIGHT || RelicRecoveryVuMark.CENTER) || System.currentTimeMillis() >= timeOut;
            //return vuMark != RelicRecoveryVuMark.UNKNOWN || System.currentTimeMillis() >= timeOut;
        }
        return true;
    }
    public void stop() {
        io.setDrivePower(0,0, 0, 0);
        execute();
    }

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
}
