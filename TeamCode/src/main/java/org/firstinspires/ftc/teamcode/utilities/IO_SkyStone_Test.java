package org.firstinspires.ftc.teamcode.utilities;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

import java.util.Locale;

import static org.firstinspires.ftc.teamcode.utilities.Sleep.sleep;

/**
 * Created by David Austin on 10/27/2016.
 */

public class IO_SkyStone_Test {
    //public DcMotor backRightMotor, backLeftMotor, frontRightMotor, frontLeftMotor, armExtenderMotor, armAngleMotor;


    public ExpansionHubEx hub2; //For RevBulkData
    public ExpansionHubEx hub3; //For RevBulkData
    public ExpansionHubMotor backRightMotor, backLeftMotor, frontRightMotor, frontLeftMotor, armExtenderMotor, armAngleMotor; //RevBulkData

    public RevBulkData bulkData2; //For RevBulkData
    public RevBulkData bulkData3; //For RevBulkData

    //public GyroSensor gyro;
    //public CRServo hook;
    public Servo rightHook;
    public Servo leftHook;
    public Servo gripperRotate;
    public Servo capStone;
    //public Servo gripperPincher; //servo
    public CRServo gripperPincher;
    public CRServo gripperPincher2;
    public DistanceSensor leftFrontDistance;
    public DistanceSensor rightFrontDistance;


    public DigitalChannel touchArmAngle;
    public DigitalChannel touchArmExtender;

    public AnalogInput armAnglePot;

    //public WebcamName webcamName;
    public VuforiaLocalizer vuforia;
    //public VuforiaLocalizer.Parameters vfparameters;

    public BNO055IMU imu;
    public BNO055IMU imu1;
    //double gyroOffset = 0;
    double imuOffset = 0;
    double imu1Offset = 0;
    public double heading = 0;
    public double heading1 = 0;
    double odometerRightOffset = 0, odometerLeftOffset = 0, odometerCenterOffset = 0, armExtenderOffset = 0, armAngleOffset = 0;
    double lastOdometerRightEncoder = 0, lastOdometerLeftEncoder = 0, lastOdometerCenterEncoder = 0, lastArmExtenderEncoder = 0, lastArmAngleEncoder = 0;
    double x = 0, y = 0, sidewaysdistance = 0;
    double x_ZeroDegree = 0, y_ZeroDegree = 0;
    //double COUNTSPERINCH = 140/1.28;//84/1.28; //used for Rev HD Hex Motor (REV-41-1301) 40:1 motor (Counts per Rotation of the Output Shaft = 1120)
    //double COUNTSPERINCH = 35;//84/1.28; //used for NeveRest Orbital 20 Gearmotor (Counts per Rotation of the Output Shaft = 537.6)
    double COUNTSPERINCH = (1440/(2 * Math.PI * 1.5)); //S4T (Counts per Rotation of the Output Shaft = 1440) with 3 inch wheels
    public double DEGREESPERVOLT = 270/3.3; //potentiometer
    public static int RED = 1, BLUE = 2;
    public static int UNKNOWN = 0, LEFT = 1, CENTER = 2, RIGHT = 3;
    public static int allianceColor = UNKNOWN;
    //public int vuMark = UNKNOWN;
    public static int cameraMonitorViewId;
    //public int jewelColor = UNKNOWN;

    //public double leftProximityAverage = 0;
    //public double rightProximityAverage = 0;
    //public double proximityCorrection = 0;
    //public boolean proximityArmButtonPushed = false;

    public double skystoneWidth = 7.5; //OpenCV
    public double backupClearance = 1.5; //OpenCV
    public double approachDistance = 3; //OpenCV
    public double distanceToTape = 22; //OpenCV
    //public double distancePastTape = 4; //OpenCV
    public double distancePastTape = 30; //OpenCV
    public double distancePastTape2 = 15; //OpenCV
    public double distanceBetweenSkystones = 24; //OpenCV

    public int skystoneDirection; //OpenCV
    public int fbskystoneDirection; //OpenCV
    public double centerOffset; //OpenCV


    public boolean skystone1Found = false; //Tensorflow
    public boolean skystone2Found = false; //Tensorflow

    public double skystoneOffsetDistanceToTape = 31; //Tensorflow

    public double skystone1Distance = 0; //Tensorflow
    public double skystone2Distance = 0; //Tensorflow

    public boolean isGoldFound = false;
    public boolean isGoldAligned = false;
    public boolean isGoldCentered = false;
    public boolean lastIsGoldFound = false;
    public boolean lastIsGoldAligned = false;
    public boolean lastIsGoldCentered = false;

    public boolean twoCyclesIsGoldFound = false;
    public boolean twoCyclesIsGoldAligned = false;
    public boolean twoCyclesIsGoldCentered = false;

    public boolean completedSearch = false;
    public boolean goldMineralFound = false;

    public boolean isGoldTheCenterMineral = false;
    public boolean isGoldTheLeftMineral = false;
    public boolean isGoldTheRightMineral = false;

    public double GoldMineralPositionCameraAverage = 0;
    public boolean GoldMineralPositionCameraAverageComplete = false;

    public double goldXPosition = 0;
    //public double headingatlanding = 0;
    public double headingOfGold = 0;

    //public static double leftHandOut = 0;
    //public static double rightHandOut = 1;
    //public static double leftHandMid = .3;
    //public static double rightHandMid = .7;
    ///public static double leftHandIn = 1;
    //public static double rightHandIn = 0;
    //public static double jewelArmUp = 1;
    //public static double jewelArmDown = 0;

    //public static double relicHandOpen = 0;
    //public static double relicHandClosed = 1;

    public static double rightHookUp = 0;
    public static double rightHookMid = .65;
    public static double rightHookMidDown = .75;
    public static double rightHookDown = 1;
    public static double leftHookUp = 1;
    public static double leftHookMid = .35;
    public static double leftHookDown = 0;
    //public static double gripperRotateStowed = 1;
    public static double gripperRotateStowed = .9;
    public static double gripperRotateParallel = .31;
    public static double gripperRotateSlightlyUp = .4;
    public static double gripperRotateDown = 0;
    public static double capStoneUp = .8;
    //public static double capStoneDown = 0;
    public static double capStoneDown = .5;

    //public static double gripperPincherOpen = 0; //servo
    //public static double gripperPincherClosed = 1; //servo


    public static double gripperPincherOpen = .85;
    public static double gripperPincherOpenSlow = .60;
    public static double gripperPincherStopped = 0;
    public static double gripperPincherClosed = -.85;
    public static double gripperPincherClosedSlow = -.60;



    public static double gripperPincher2Open = -.85;
    public static double gripperPincher2OpenSlow = -.60;
    public static double gripperPincher2Stopped = 0;
    public static double gripperPincher2Closed = .85;
    public static double gripperPincher2ClosedSlow = .60;

    // State used for updating telemetry
    public Orientation angles;
    public Acceleration gravity;
    public Orientation angles1;
    public Acceleration gravity1;
    public double[] eulerAngles = new double[3];
    public double[] eulerAngles1 = new double[3];

    // Set up the parameters with which we will use our IMU. Note that integration
    // algorithm here just reports accelerations to the logcat log; it doesn't actually
    // provide positional information.
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    Telemetry telemetry;
    HardwareMap map;

    public IO_SkyStone_Test(HardwareMap map, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.map = map;

        hub2 = map.get(ExpansionHubEx.class, "Expansion Hub 2"); //For RevBulkData
        hub3 = map.get(ExpansionHubEx.class, "Expansion Hub 3"); //For RevBulkData

        //backRightMotor = map.dcMotor.get("motorBR");
        //backLeftMotor = map.dcMotor.get("motorBL");
        //frontRightMotor = map.dcMotor.get("motorFR");
        //frontLeftMotor = map.dcMotor.get("motorFL");
        //armExtenderMotor = map.dcMotor.get("motorAE");
        //armAngleMotor = map.dcMotor.get("motorAA");

        backRightMotor = map.get(ExpansionHubMotor.class, "motorBR"); //For RevBulkData
        backLeftMotor = map.get(ExpansionHubMotor.class, "motorBL"); //For RevBulkData
        frontRightMotor = map.get(ExpansionHubMotor.class, "motorFR"); //For RevBulkData
        frontLeftMotor = map.get(ExpansionHubMotor.class, "motorFL"); //For RevBulkData
        armExtenderMotor = map.get(ExpansionHubMotor.class, "motorAE"); //For RevBulkData
        armAngleMotor = map.get(ExpansionHubMotor.class, "motorAA"); //For RevBulkData

        //hook = map.crservo.get("hook");
        rightHook = map.servo.get("servoRH");
        leftHook = map.servo.get("servoLH");
        gripperRotate = map.servo.get("servoGR");
        capStone = map.servo.get("servoCS");
        //gripperPincher = map.servo.get("servoGP"); //servo
        gripperPincher = map.crservo.get("servoGP");
        gripperPincher2 = map.crservo.get("servoGP2");
        /*rightHand = map.servo.get("right_hand");
        jewelArm = map.servo.get("jewel_arm");
        proximityArm = map.servo.get("proximity_arm");
        relicHand = map.servo.get("relic_hand");*/

        touchArmAngle = map.digitalChannel.get("touchAA");
        touchArmExtender = map.digitalChannel.get("touchAE");

        armAnglePot = map.analogInput.get("potAA");

        //webcamName = map.get(WebcamName.class, "Webcam 1");
        /*touchProximity = map.digitalChannel.get("touchproximity");
        touchLowerRelicArm = map.digitalChannel.get("touchlowerrelicarm");
        touchUpperRelicArm = map.digitalChannel.get("touchupperrelicarm");*/

/*        String VUFORIA_KEY =
                "Ae+uGTX/////AAABmQV3De8djUCDjn2zDZDbCssvJv8/irA8Dzm+UnPYeGcgN7Y/V1EFU/DgmBcA3x5TxqeooD4B02M6PR+5IBifNlYVIXezFdgl/f9PKHDE7KAl3yeEV993njRk8ocjpNJwYDqcN1vZP6yWRqe4Y9QdAJH+KZPQeR+eN5wT87m4ZNHhsC5DidIkFYuhVNVdM+Gn9CLUphmjX1woXqSLqK3BdmU6XEfKU730USi7clKwVidBUMCcFcL878gUG0Mn5JL7dcPUO3r1q+8ODt1wInwPWgSQlXrrY4wWSeHJ5VwwihGnisIZ2Ps41yqf1QtrzK7FsDz5P5aQaQ7rVtzntFLZZ+ftIy0aJ+YelBy1QtZX+dc8";*/
        //vfparameters = new VuforiaLocalizer.Parameters();
        //vfparameters.vuforiaLicenseKey = VUFORIA_KEY;
        //vfparameters.cameraName = webcamName;
        //vuforia = ClassFactory.getInstance().createVuforia(vfparameters);


        //gyro = map.gyroSensor.get("gyro");
        imu = map.get(BNO055IMU.class, "imu");
        imu1 = map.get(BNO055IMU.class, "imu1");

        /*//ods = map.opticalDistanceSensor.get("ods");
        colorSensor = map.colorSensor.get("colorsensor");*/


        /*// get a reference to the color sensor.
        leftColor = map.get(ColorSensor.class, "leftcolor");*/

        // get a reference to the distance sensor that shares the same name.
        leftFrontDistance = map.get(DistanceSensor.class, "DL");
        rightFrontDistance = map.get(DistanceSensor.class, "DR");
        //frontDistance = map.get(DistanceSensor.class, "frontdistance");
        //backDistance = map.get(DistanceSensor.class, "backdistance");

        /*// get a reference to the color sensor.
        rightColor = map.get(ColorSensor.class, "rightcolor");

        // get a reference to the distance sensor that shares the same name.
        rightDistance = map.get(DistanceSensor.class, "rightcolor");

        // get a reference to the color sensor.
        revColorSensor = map.get(ColorSensor.class, "jewelcolor");

        // get a reference to the distance sensor that shares the same name.
        jewelDistance = map.get(DistanceSensor.class, "jewelcolor");*/


        touchArmAngle.setMode(DigitalChannel.Mode.INPUT);
        touchArmExtender.setMode(DigitalChannel.Mode.INPUT);
        /*touchProximity.setMode(DigitalChannel.Mode.INPUT);
        touchLowerRelicArm.setMode(DigitalChannel.Mode.INPUT);
        touchUpperRelicArm.setMode(DigitalChannel.Mode.INPUT);*/

        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE); //REV 20:1
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD); //REV 20:1
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE); //REV 20:1
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD); //REV 20:1


        //backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD); //REV 40:1
        //backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE); //REV 40:1
        //frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD); //REV 40:1
        //frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE); //REV 40:1
        armExtenderMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armAngleMotor.setDirection(DcMotorSimple.Direction.REVERSE);

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

    /*public void retractHands() {
        leftHand.setPosition(IO_SkyStone_Test.leftHandOut);
        rightHand.setPosition(IO_SkyStone_Test.rightHandOut);
    }

    public void retractHandsMid() {
        leftHand.setPosition(IO_SkyStone_Test.leftHandMid);
        rightHand.setPosition(IO_SkyStone_Test.rightHandMid);
    }

    public void closeHands() {
        leftHand.setPosition(IO_SkyStone_Test.leftHandIn);
        rightHand.setPosition(IO_SkyStone_Test.rightHandIn);
    }*/

    public void rightHookUp() { rightHook.setPosition(IO_SkyStone_Test.rightHookUp); }
    public void rightHookMid() { rightHook.setPosition(IO_SkyStone_Test.rightHookMid); }
    public void rightHookMidDown() { rightHook.setPosition(IO_SkyStone_Test.rightHookMidDown); }
    public void rightHookDown() { rightHook.setPosition(IO_SkyStone_Test.rightHookDown); }
    public void leftHookUp() { leftHook.setPosition(IO_SkyStone_Test.leftHookUp); }
    public void leftHookMid() { leftHook.setPosition(IO_SkyStone_Test.leftHookMid); }
    public void leftHookDown() { leftHook.setPosition(IO_SkyStone_Test.leftHookDown); }
    public void gripperRotateStowed() { gripperRotate.setPosition(IO_SkyStone_Test.gripperRotateStowed); }
    public void gripperRotateParallel() { gripperRotate.setPosition(IO_SkyStone_Test.gripperRotateParallel); }
    public void setGripperRotateSlightlyUp() { gripperRotate.setPosition(IO_SkyStone_Test.gripperRotateSlightlyUp); }
    public void gripperRotateDown() { gripperRotate.setPosition(IO_SkyStone_Test.gripperRotateDown); }
    public void capStoneUp() { capStone.setPosition(IO_SkyStone_Test.capStoneUp); }
    public void capStoneDown() { capStone.setPosition(IO_SkyStone_Test.capStoneDown); }

    //public void gripperPincherOpen() { gripperPincher.setPosition(IO_SkyStone_Test.gripperPincherOpen); } //servo
    //public void gripperPincherClosed() { gripperPincher.setPosition(IO_SkyStone_Test.gripperPincherClosed); } //servo

    public void gripperPincherOpen() { gripperPincher.setPower(IO_SkyStone_Test.gripperPincherOpen); }
    public void gripperPincherOpenSlow() { gripperPincher.setPower(IO_SkyStone_Test.gripperPincherOpenSlow); }
    public void gripperPincherClosed() { gripperPincher.setPower(IO_SkyStone_Test.gripperPincherClosed); }
    public void gripperPincherClosedSlow() { gripperPincher.setPower(IO_SkyStone_Test.gripperPincherClosedSlow); }
    public void gripperPincherStopped() { gripperPincher.setPower(IO_SkyStone_Test.gripperPincherStopped); }

    public void gripperPincher2Open() { gripperPincher2.setPower(IO_SkyStone_Test.gripperPincher2Open); }
    public void gripperPincher2OpenSlow() { gripperPincher2.setPower(IO_SkyStone_Test.gripperPincher2OpenSlow); }
    public void gripperPincher2Closed() { gripperPincher2.setPower(IO_SkyStone_Test.gripperPincher2Closed); }
    public void gripperPincher2ClosedSlow() { gripperPincher2.setPower(IO_SkyStone_Test.gripperPincher2ClosedSlow); }
    public void gripperPincher2Stopped() { gripperPincher2.setPower(IO_SkyStone_Test.gripperPincher2Stopped); }


    /*public void jewelArmDown() { jewelArm.setPosition(IO_SkyStone_Test.jewelArmDown); }

    public void proximityArmUp() {
        proximityArm.setPosition(IO_SkyStone_Test.proximityArmUp);
    }

    public void proximityArmMid() {
        proximityArm.setPosition(IO_SkyStone_Test.proximityArmMid);
    }*/

    /*public void proximityArmDown() {
        proximityArm.setPosition(IO_SkyStone_Test.proximityArmDown);
    }*/

/*    public void setGyroOffset() {
        gyroOffset = gyro.getHeading();
    }*/
    public void setIMUOffset() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        imuOffset = angles.firstAngle;
    }

    public void setIMU1Offset() {
        angles1 = imu1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        imu1Offset = angles1.firstAngle;
    }

    public void resetDriveEncoders() {

        bulkData2 = hub2.getBulkInputData(); //For RevBulkData
        bulkData3 = hub3.getBulkInputData(); //For RevBulkData

        odometerRightOffset = bulkData2.getMotorCurrentPosition(frontLeftMotor); //For RevBulkData
        odometerLeftOffset = bulkData3.getMotorCurrentPosition(frontRightMotor); //For RevBulkData
        odometerCenterOffset = bulkData3.getMotorCurrentPosition(backRightMotor); //For RevBulkData
        armExtenderOffset = bulkData3.getMotorCurrentPosition(armExtenderMotor); //For RevBulkData
        armAngleOffset = bulkData3.getMotorCurrentPosition(armAngleMotor); //For RevBulkData

        //odometerRightOffset = frontLeftMotor.getCurrentPosition();
        //odometerLeftOffset = frontRightMotor.getCurrentPosition();
        //odometerCenterOffset = backRightMotor.getCurrentPosition();
        //armExtenderOffset = armExtenderMotor.getCurrentPosition();
        //armAngleOffset = armAngleMotor.getCurrentPosition();

        lastOdometerRightEncoder = 0;
        lastOdometerLeftEncoder = 0;
        lastOdometerCenterEncoder = 0;
        lastArmExtenderEncoder = 0;
        lastArmAngleEncoder = 0;

        x = 0;
        y = 0;
        sidewaysdistance = 0;
        x_ZeroDegree = 0;
        y_ZeroDegree = 0;
    }
    public void updatePosition() {
        bulkData2 = hub2.getBulkInputData(); //For RevBulkData
        bulkData3 = hub3.getBulkInputData(); //For RevBulkData

        double odometerRightEncoder = getOdometerRightEncoder();
        double odometerLeftEncoder = getOdometerLeftEncoder();
        double odometerCenterEncoder = getOdometerCenterEncoder();
        double armExtenderEncoder = getArmExtenderEncoder();
        double armAngleEncoder = getArmAngleEncoder();

        //double dom1Encoder = getDOM1MotorEncoder();
        //double dom2Encoder = getDOM2MotorEncoder();
        //double domExtendEncoder = getDOMMotorExtendEncoder();

        //double averageChange = (leftBackEncoder - lastLeftBackEncoder);
        double averageChange = ((odometerLeftEncoder - lastOdometerLeftEncoder) + (odometerRightEncoder - lastOdometerRightEncoder))/2.0;
        double averageChangeSideways = (odometerCenterEncoder - lastOdometerCenterEncoder);


        /*if (isGoldFound && lastIsGoldFound) {
            twoCyclesIsGoldFound = true;
        } else {
            twoCyclesIsGoldFound = false;
        }

        if (isGoldAligned && lastIsGoldAligned) {
            twoCyclesIsGoldAligned = true;
        } else {
            twoCyclesIsGoldAligned = false;
        }

        if (isGoldCentered && lastIsGoldCentered) {
            twoCyclesIsGoldCentered = true;
        } else {
            twoCyclesIsGoldCentered = false;
        }*/

        //gravity  = imu.getGravity();
        //getIMUHeading();
        //getEulerAngles(eulerAngles);;

        //heading from gyro
        //double heading = Math.toRadians(getHeading());
        //heading from imu
        heading = Math.toRadians(getIMUHeading());
        //heading1 = Math.toRadians(getIMU1Heading());
        //x += averageChange * Math.cos(heading);
        //y += averageChange * Math.sin(heading);

        x += averageChange;
        y += averageChange;
        sidewaysdistance += averageChangeSideways;



        x_ZeroDegree += averageChange * Math.cos(0);
        y_ZeroDegree += averageChange * Math.sin(0);

        lastOdometerRightEncoder = odometerRightEncoder;
        lastOdometerLeftEncoder = odometerLeftEncoder;
        lastOdometerCenterEncoder = odometerCenterEncoder;
        lastArmExtenderEncoder = armExtenderEncoder;
        lastArmAngleEncoder = armAngleEncoder;

        /*lastIsGoldFound = isGoldFound;
        lastIsGoldAligned = isGoldAligned;
        lastIsGoldCentered = isGoldCentered;*/

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

        telemetry.addData("IMU Heading",  "Starting at %.2f",
                getIMUHeading());
        /*telemetry.addData("IMU1 Heading",  "Starting at %.2f",
                getIMU1Heading());*/

/*        telemetry.addData("IMU Y",  "Starting at %.2f",
                angles.secondAngle);
        telemetry.addData("IMU1 Y",  "Starting at %.2f",
                angles.secondAngle);

        telemetry.addData("IMU X",  "Starting at %.2f",
                angles.thirdAngle);
        telemetry.addData("IMU1 X",  "Starting at %.2f",
                angles.thirdAngle);*/

    }

    public double getX() {
        return -x / COUNTSPERINCH;
    }
    public double getY() {
        return -y / COUNTSPERINCH;
    }
    public double getSidewaysDistance() { return -sidewaysdistance / COUNTSPERINCH; }

    public double getX_ZeroDegree() {
        return x_ZeroDegree / COUNTSPERINCH;
    }
    public double getY_ZeroDegree() {
        return y_ZeroDegree / COUNTSPERINCH;
    }

/*    public double getHeading() {
        double gyroheading = gyro.getHeading() - gyroOffset;
        while (gyroheading > 180) {
            gyroheading -= 360;
        }
        while (gyroheading <= - 180) {
            gyroheading += 360;
        }
        return -gyroheading;
    }*/

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

    public double getIMU1Heading() {
        // Acquiring the angles is relatively expensive; we don't want
        // to do that in each of the three items that need that info, as that's
        // three times the necessary expense.
        angles1   = imu1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double imu1Heading = angles1.firstAngle - imu1Offset;
        while (imu1Heading > 180) {
            imu1Heading -= 360;
        }
        while (imu1Heading <= - 180) {
            imu1Heading += 360;
        }
        return -imu1Heading;
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

    private void getEulerAngles1(double[] angles1)
    {
        Quaternion q1 = imu1.getQuaternionOrientation();

        //
        // 0: roll (x-axis rotation)
        // 1: pitch (y-axis rotation)
        // 2: yaw (z-axis rotation)
        //
        angles1[0] = Math.toDegrees(Math.atan2(2.0*(q1.w*q1.x + q1.y*q1.z), 1.0 - 2.0*(q1.x*q1.x + q1.y*q1.y)));
        double sinp = 2.0*(q1.w*q1.y - q1.z*q1.x);
        angles1[1] = Math.toDegrees(Math.abs(sinp) >= 1.0? Math.signum(sinp)*(Math.PI/2.0): Math.asin(sinp));
        angles1[2] = Math.toDegrees(Math.atan2(2.0*(q1.w*q1.z + q1.x*q1.y), 1.0 - 2.0*(q1.y*q1.y + q1.z*q1.z)));
    }

    public void normalize(double[] wheelPowers)
    {
        double maxMagnitude = Math.abs(wheelPowers[0]);

        for (int i = 1; i < wheelPowers.length; i++)
        {
            double magnitude = Math.abs(wheelPowers[i]);

            if (magnitude > maxMagnitude)
            {
                maxMagnitude = magnitude;
            }
        }

        if (maxMagnitude > 1.0)
        {
            for (int i = 0; i < wheelPowers.length; i++)
            {
                wheelPowers[i] /= maxMagnitude;
            }
        }
    }   //normalize


    public double getOdometerRightEncoder() {
        return bulkData2.getMotorCurrentPosition(frontLeftMotor) - odometerRightOffset; //For RevBulkData
        //return frontLeftMotor.getCurrentPosition() - odometerRightOffset;
    }

    public double getOdometerLeftEncoder() {
        return bulkData3.getMotorCurrentPosition(frontRightMotor) - odometerLeftOffset; //For RevBulkData
        //return frontRightMotor.getCurrentPosition() - odometerLeftOffset;
    }

    public double getOdometerCenterEncoder() {
        return bulkData3.getMotorCurrentPosition(backRightMotor) - odometerCenterOffset; //For RevBulkData
        //return backRightMotor.getCurrentPosition() - odometerCenterOffset;
    }

    public double getArmExtenderEncoder() {
        return bulkData3.getMotorCurrentPosition(armExtenderMotor) - armExtenderOffset; //For RevBulkData
        //return armExtenderMotor.getCurrentPosition() - armExtenderOffset;
    }

    public double getArmAngleEncoder() {
        return bulkData3.getMotorCurrentPosition(armAngleMotor) - armAngleOffset; //For RevBulkData
        //return armAngleMotor.getCurrentPosition() - armAngleOffset;
    }


    /*public double getDOM1MotorEncoder() {
        return dom1Motor.getCurrentPosition() - dom1Offset;
    }

    public double getDOM2MotorEncoder() {
        return dom2Motor.getCurrentPosition() - dom2Offset;
    }

    public double getDOMMotorExtendEncoder() {
        return domExtendMotor.getCurrentPosition() - domExtendOffset;
    }*/

    public double getArmAnglePotVoltage() {
        return armAnglePot.getVoltage();
    }

    public double getArmAnglePotDegrees() {
        return (270 - (armAnglePot.getVoltage()*DEGREESPERVOLT));
    }

    public double getGoldXPosition() {
        return (goldXPosition);
    }
    public double getGoldXPositionAroundZero() {
        return (goldXPosition - 320);
    }


    public void setDrivePower(double fl, double fr, double bl, double br) {
        frontLeftMotor.setPower(fl);
        frontRightMotor.setPower(fr);
        backLeftMotor.setPower(bl);
        backRightMotor.setPower(br);
    }

    /*public double getLightReading(){
        return ods.getLightDetected();
    }*/

    public void calibrateGyroandIMU() {
        // start calibrating the gyro.
        telemetry.addData(">", "IMU Calibrating. Do Not Move!");
        //telemetry.update();

        //sleep(500);

        //Thread.sleep(1000); // wait 1 second for gyro to stabilize (may be movement from initializing servo)
        //gyro.calibrate();
        imu.initialize(parameters);
    }

    public void calibrateGyroandIMU1() {
        // start calibrating the gyro.
        telemetry.addData(">", "IMU1 Calibrating. Do Not Move!");
        //telemetry.update();

        //sleep(500);

        //Thread.sleep(1000); // wait 1 second for gyro to stabilize (may be movement from initializing servo)
        //gyro.calibrate();
        imu1.initialize(parameters);
    }

    /*public void setJewelColor(int jcolor){
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
    }*/

    public void setAllianceColor(int acolor){
        allianceColor = acolor;
    }
    public int getAllianceColor(){
        return allianceColor;
    }

    /*public void setVuMark(int location){
        vuMark = location;
    }
    public int getVuMark(){
        return vuMark;
    }*/

    /*public void setHook(double open_close) {
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
    }*/

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
