package org.firstinspires.ftc.teamcode.utilities;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static org.firstinspires.ftc.teamcode.utilities.Sleep.sleep;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.robotcore.external.navigation.TempUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

/**
 * Created by David Austin on 10/27/2016.
 */

public class IO_4WD_Test {
    public DcMotor rightBackDrive, leftBackDrive, rightFrontDrive, leftFrontDrive, forkLiftMotor, rpu1Motor, rpu2Motor;
    public GyroSensor gyro;
    public Servo leftHand, rightHand, jewelArm, proximityArm, relicHand;
    public OpticalDistanceSensor ods;
    //public TouchSensor touchBottom, touchProximity, touchLowerRelicArm, touchUpperRelicArm;
    public DigitalChannel touchBottom, touchProximity, touchLowerRelicArm, touchUpperRelicArm;
    public ColorSensor colorSensor;

    public ColorSensor leftColor;
    public DistanceSensor leftDistance;
    public ColorSensor rightColor;
    public DistanceSensor rightDistance;

    public ColorSensor revColorSensor;
    public DistanceSensor jewelDistance;

    public BNO055IMU imu;
    double gyroOffset = 0;
    double imuOffset = 0;
    public double heading = 0;
    double rightBackOffset = 0, leftBackOffset = 0, rightFrontOffset = 0, leftFrontOffset = 0, forkLiftOffset = 0, rpu1Offset = 0, rpu2Offset = 0;
    double lastRightBackEncoder = 0, lastLeftBackEncoder = 0, lastRightFrontEncoder = 0, lastLeftFrontEncoder = 0, lastForkLiftEncoder = 0, lastRPU1Encoder = 0, lastRPU2Encoder = 0;
    double x = 0, y = 0;
    double COUNTSPERINCH = 140/1.28;//84/1.28;
    public static int RED = 1, BLUE = 2;
    public static int UNKNOWN = 0, LEFT = 1, CENTER = 2, RIGHT = 3;
    public static int allianceColor = UNKNOWN;
    public int vuMark = UNKNOWN;
    public static int cameraMonitorViewId;
    public int jewelColor = UNKNOWN;

    public double leftProximityAverage = 0;
    public double rightProximityAverage = 0;
    public double proximityCorrection = 0;
    public boolean proximityArmButtonPushed = false;

    public static double proximityArmUp = 1;
    public static double proximityArmMid = 0.6;
    public static double proximityArmDown = 0;

    public static double leftHandOut = 0;
    public static double rightHandOut = 1;
    public static double leftHandMid = .3;
    public static double rightHandMid = .7;
    public static double leftHandIn = 1;
    public static double rightHandIn = 0;
    public static double jewelArmUp = 1;
    public static double jewelArmDown = 0;

    public static double relicHandOpen = 0;
    public static double relicHandClosed = 1;

    // State used for updating telemetry
    public Orientation angles;
    public Acceleration gravity;
    public double[] eulerAngles = new double[3];

    // Set up the parameters with which we will use our IMU. Note that integration
    // algorithm here just reports accelerations to the logcat log; it doesn't actually
    // provide positional information.
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    Telemetry telemetry;

    public IO_4WD_Test(HardwareMap map, Telemetry telemetry) {
        this.telemetry = telemetry;
        rightBackDrive = map.dcMotor.get("backright");
        leftBackDrive = map.dcMotor.get("backleft");
        rightFrontDrive = map.dcMotor.get("frontright");
        leftFrontDrive = map.dcMotor.get("frontleft");
        forkLiftMotor = map.dcMotor.get("fork_lift_motor");
        rpu1Motor = map.dcMotor.get("rpu1_motor");
        rpu2Motor = map.dcMotor.get("rpu2_motor");

        leftHand = map.servo.get("left_hand");
        rightHand = map.servo.get("right_hand");
        jewelArm = map.servo.get("jewel_arm");
        proximityArm = map.servo.get("proximity_arm");
        relicHand = map.servo.get("relic_hand");

        touchBottom = map.digitalChannel.get("touchbottom");
        touchProximity = map.digitalChannel.get("touchproximity");
        touchLowerRelicArm = map.digitalChannel.get("touchlowerrelicarm");
        touchUpperRelicArm = map.digitalChannel.get("touchupperrelicarm");

        gyro = map.gyroSensor.get("gyro");
        imu = map.get(BNO055IMU.class, "imu");

        //ods = map.opticalDistanceSensor.get("ods");
        colorSensor = map.colorSensor.get("colorsensor");


        // get a reference to the color sensor.
        leftColor = map.get(ColorSensor.class, "leftcolor");

        // get a reference to the distance sensor that shares the same name.
        leftDistance = map.get(DistanceSensor.class, "leftcolor");

        // get a reference to the color sensor.
        rightColor = map.get(ColorSensor.class, "rightcolor");

        // get a reference to the distance sensor that shares the same name.
        rightDistance = map.get(DistanceSensor.class, "rightcolor");

        // get a reference to the color sensor.
        revColorSensor = map.get(ColorSensor.class, "jewelcolor");

        // get a reference to the distance sensor that shares the same name.
        jewelDistance = map.get(DistanceSensor.class, "jewelcolor");

        touchBottom.setMode(DigitalChannel.Mode.INPUT);
        touchProximity.setMode(DigitalChannel.Mode.INPUT);
        touchLowerRelicArm.setMode(DigitalChannel.Mode.INPUT);
        touchUpperRelicArm.setMode(DigitalChannel.Mode.INPUT);

        rightBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        forkLiftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rpu1Motor.setDirection(DcMotorSimple.Direction.REVERSE);
        rpu2Motor.setDirection(DcMotorSimple.Direction.FORWARD);

        cameraMonitorViewId = map.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", map.appContext.getPackageName());

        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.temperatureUnit     = BNO055IMU.TempUnit.FARENHEIT;
        //parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.mode = BNO055IMU.SensorMode.IMU;
        //parameters.pitchMode = BNO055IMU.PitchMode.WINDOWS;
        parameters.useExternalCrystal   = true;
        //parameters.loggingEnabled      = true;
        //parameters.loggingTag          = "IMU";
        //parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
    }

    public void retractHands() {
        leftHand.setPosition(IO_4WD_Test.leftHandOut);
        rightHand.setPosition(IO_4WD_Test.rightHandOut);
    }

    public void retractHandsMid() {
        leftHand.setPosition(IO_4WD_Test.leftHandMid);
        rightHand.setPosition(IO_4WD_Test.rightHandMid);
    }

    public void closeHands() {
        leftHand.setPosition(IO_4WD_Test.leftHandIn);
        rightHand.setPosition(IO_4WD_Test.rightHandIn);
    }

    public void openRelicHand() {
        relicHand.setPosition(IO_4WD_Test.relicHandOpen);
    }

    public void closeRelicHand() {
        relicHand.setPosition(IO_4WD_Test.relicHandClosed);
    }

    public void jewelArmUp() { jewelArm.setPosition(IO_4WD_Test.jewelArmUp); }

    public void jewelArmDown() { jewelArm.setPosition(IO_4WD_Test.jewelArmDown); }

    public void proximityArmUp() {
        proximityArm.setPosition(IO_4WD_Test.proximityArmUp);
    }

    public void proximityArmMid() {
        proximityArm.setPosition(IO_4WD_Test.proximityArmMid);
    }

    public void proximityArmDown() {
        proximityArm.setPosition(IO_4WD_Test.proximityArmDown);
    }

    public void setGyroOffset() {
        gyroOffset = gyro.getHeading();
    }
    public void setIMUOffset() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        imuOffset = angles.firstAngle;
    }

    public void resetDriveEncoders() {
        rightBackOffset = rightBackDrive.getCurrentPosition();
        leftBackOffset = leftBackDrive.getCurrentPosition();
        rightFrontOffset = rightFrontDrive.getCurrentPosition();
        leftFrontOffset = leftFrontDrive.getCurrentPosition();

        forkLiftOffset = forkLiftMotor.getCurrentPosition();
        rpu1Offset = rpu1Motor.getCurrentPosition();
        rpu2Offset = rpu2Motor.getCurrentPosition();

        lastRightBackEncoder = 0;
        lastLeftBackEncoder = 0;
        lastRightFrontEncoder = 0;
        lastLeftFrontEncoder = 0;

        lastForkLiftEncoder = 0;
        lastRPU1Encoder = 0;
        lastRPU2Encoder = 0;

        x = 0;
        y = 0;
    }
    public void updatePosition() {
        double rightBackEncoder = getRightBackDriveEncoder();
        double leftBackEncoder = getLeftBackDriveEncoder();
        double rightFrontEncoder = getRightFrontDriveEncoder();
        double leftFrontEncoder = getLeftFrontDriveEncoder();

        double forkLiftEncoder = getForkLiftMotorEncoder();
        double rpu1Encoder = getRPU1MotorEncoder();
        double rpu2Encoder = getRPU2MotorEncoder();

        //double averageChange = (leftBackEncoder - lastLeftBackEncoder);
        double averageChange = (leftBackEncoder - lastLeftBackEncoder + rightFrontEncoder - lastRightFrontEncoder)/2.0;

        //gravity  = imu.getGravity();
        //getIMUHeading();
        //getEulerAngles(eulerAngles);;

        //heading from gyro
        //double heading = Math.toRadians(getHeading());
        //heading from imu
        heading = Math.toRadians(getIMUHeading());
        x += averageChange * Math.cos(heading);
        y += averageChange * Math.sin(heading);
        lastRightBackEncoder = rightBackEncoder;
        lastLeftBackEncoder = leftBackEncoder;
        lastRightFrontEncoder = rightFrontEncoder;
        lastLeftFrontEncoder = leftFrontEncoder;

        lastForkLiftEncoder = forkLiftEncoder;
        lastRPU1Encoder = rpu1Encoder;
        lastRPU2Encoder = rpu2Encoder;

        //lastRightBackEncoder = rightBackEncoder;
        //lastLeftBackEncoder = leftBackEncoder;
        //lastRightFrontEncoder = rightFrontEncoder;
        //lastLeftFrontEncoder = leftFrontEncoder;
        //telemetry.addData("Gyro Heading:", gyro.getHeading());
        //telemetry.addData("Gyro IO Heading: ", getHeading());

        //telemetry.addData("IMU Heading:", angles.firstAngle);
        //telemetry.addData("IMU IO Heading: ", Math.toDegrees(heading));

        /*telemetry.addData("Distance left sensor (cm)",
                String.format(Locale.US, "%.02f", leftDistance.getDistance(DistanceUnit.CM)));

        telemetry.addData("Distance right sensor(cm)",
                String.format(Locale.US, "%.02f", rightDistance.getDistance(DistanceUnit.CM)));*/

    }

    public double getX() {
        return x / COUNTSPERINCH;
    }
    public double getY() {
        return y / COUNTSPERINCH;
    }

    public double getHeading() {
        double gyroheading = gyro.getHeading() - gyroOffset;
        while (gyroheading > 180) {
            gyroheading -= 360;
        }
        while (gyroheading <= - 180) {
            gyroheading += 360;
        }
        return -gyroheading;
    }

    public double getIMUHeading() {
        // Acquiring the angles is relatively expensive; we don't want
        // to do that in each of the three items that need that info, as that's
        // three times the necessary expense.
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double imuHeading = angles.firstAngle - imuOffset;
        while (imuHeading > 180) {
            imuHeading -= 360;
        }
        while (imuHeading <= - 180) {
            imuHeading += 360;
        }
        return -imuHeading;
    }

    /**
     * This method returns the Euler angles of all 3 axes from quaternion orientation.
     *
     * @param angles specifies the array to hold the angles of the 3 axes.
     */
    private void getEulerAngles(double[] angles)
    {
        Quaternion q = imu.getQuaternionOrientation();

        //
        // 0: roll (x-axis rotation)
        // 1: pitch (y-axis rotation)
        // 2: yaw (z-axis rotation)
        //
        angles[0] = Math.toDegrees(Math.atan2(2.0*(q.w*q.x + q.y*q.z), 1.0 - 2.0*(q.x*q.x + q.y*q.y)));
        double sinp = 2.0*(q.w*q.y - q.z*q.x);
        angles[1] = Math.toDegrees(Math.abs(sinp) >= 1.0? Math.signum(sinp)*(Math.PI/2.0): Math.asin(sinp));
        angles[2] = Math.toDegrees(Math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z)));
    }   //getEulerAngles

    public double getRightBackDriveEncoder() {
        return rightBackDrive.getCurrentPosition() - rightBackOffset;
    }
    public double getLeftBackDriveEncoder() {
        return leftBackDrive.getCurrentPosition() - leftBackOffset;
    }
    public double getRightFrontDriveEncoder() {
        return rightFrontDrive.getCurrentPosition() - rightFrontOffset; }

    public double getLeftFrontDriveEncoder() {
        return leftFrontDrive.getCurrentPosition() - leftFrontOffset;
    }

    public double getForkLiftMotorEncoder() {
        return forkLiftMotor.getCurrentPosition() - forkLiftOffset;
    }

    public double getRPU1MotorEncoder() {
        return rpu1Motor.getCurrentPosition() - rpu1Offset;
    }
    public double getRPU2MotorEncoder() {
        return rpu2Motor.getCurrentPosition() - rpu2Offset;
    }

    public void setDrivePower(double left, double right) {
        rightBackDrive.setPower(right);
        leftBackDrive.setPower(left);
        rightFrontDrive.setPower(right);
        leftFrontDrive.setPower(left);
    }

    public double getLightReading(){
        return ods.getLightDetected();
    }

    public void calibrateGyroandIMU() {
        // start calibrating the gyro.
        telemetry.addData(">", "IMU Calibrating. Do Not Move!");
        //telemetry.update();

        sleep(500);

        //Thread.sleep(1000); // wait 1 second for gyro to stabilize (may be movement from initializing servo)
        //gyro.calibrate();
        imu.initialize(parameters);
    }

    public void setJewelColor(int jcolor){
        jewelColor = jcolor;
    }
    public int getJewelColor(){
        return jewelColor;
    }

    public void setLeftProximityAverage(double leftproximityaverage){
        leftProximityAverage = leftproximityaverage;
    }
    public double getLeftProximityAverage(){
        return leftProximityAverage;
    }

    public void setRightProximityAverage(double rightproximityaverage){
        rightProximityAverage = rightproximityaverage;
    }
    public double getRightProximityAverage(){
        return rightProximityAverage;
    }

    public double getProximityCorrection(){
        if (leftProximityAverage > 20){
            //proximityCorrection = 20;
            proximityCorrection = 5;
        } else if((leftProximityAverage >= 7) && (leftProximityAverage <= 11)) {
            proximityCorrection = 0;
        } else if(leftProximityAverage > 11) {
            //proximityCorrection = (((20 * leftProximityAverage) / 13) - (140/13));
            proximityCorrection = (((5.0 * leftProximityAverage) / 9.0) - (55.0/9.0));
        } else if (rightProximityAverage > 20) {
            //proximityCorrection = -20;
            proximityCorrection = -5;
        } else if((rightProximityAverage >= 7) && (rightProximityAverage <= 11)) {
            proximityCorrection = 0;
        } else if (rightProximityAverage > 11) {
            //proximityCorrection = -(((20 * rightProximityAverage) / 13) - (140 / 13));
            proximityCorrection = -(((5.0 * rightProximityAverage) / 9.0) - (55.0/9.0));
        } else {
            proximityCorrection = 0;
        }
        return -proximityCorrection;
    }

    public void setAllianceColor(int acolor){
        allianceColor = acolor;
    }
    public int getAllianceColor(){
        return allianceColor;
    }

    public void setVuMark(int location){
        vuMark = location;
    }
    public int getVuMark(){
        return vuMark;
    }

    public void setHands(double open_close) {
        leftHand.setPosition(open_close);
        rightHand.setPosition(1 - open_close);
    }

    public void setRelicHand(double position) {
        relicHand.setPosition(1 - position);
    }

    public void setJewelArm(double position) {
        jewelArm.setPosition(1 - position);
    }

    public void setProximityArm(double position) {
        proximityArm.setPosition(1 - position);
    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

}
