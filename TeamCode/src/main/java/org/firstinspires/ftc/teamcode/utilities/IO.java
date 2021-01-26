package org.firstinspires.ftc.teamcode.utilities;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static org.firstinspires.ftc.teamcode.utilities.Sleep.sleep;

/**
 * Created by David Austin on 10/27/2016.
 */

public class IO {
    public DcMotor leftDrive, rightDrive, arm, forkLiftMotor;
    public GyroSensor gyro;
    public Servo leftHand, rightHand;
    public OpticalDistanceSensor ods;
    public TouchSensor touchBottom, touchTop;
    public ColorSensor colorSensor;
    double gyroOffset = 0;
    double leftOffset = 0, rightOffset = 0, forkLiftOffset = 0;
    double lastLeftEncoder = 0, lastRightEncoder = 0, lastForkLiftEncoder = 0;
    double x = 0, y = 0;
    double COUNTSPERINCH = 54/1.28;//84/1.28;
    public static int RED = 1, BLUE = 2;
    public static int UNKNOWN = 0, LEFT = 1, CENTER = 2, RIGHT = 3;
    public int allianceColor = UNKNOWN;
    public int vuMark = UNKNOWN;
    public int cameraMonitorViewId;
    public int jewelColor = UNKNOWN;

    public static int leftHandOut = 0;
    public static int rightHandOut = 1;

    Telemetry telemetry;

    public IO(HardwareMap map, Telemetry telemetry) {
        this.telemetry = telemetry;
        leftDrive = map.dcMotor.get("left_drive");
        rightDrive = map.dcMotor.get("right_drive");
        arm = map.dcMotor.get("arm");
        forkLiftMotor = map.dcMotor.get("fork_lift_motor");

        leftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        forkLiftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        arm.setDirection(DcMotorSimple.Direction.FORWARD);

        gyro = map.gyroSensor.get("gyro");
        ods = map.opticalDistanceSensor.get("ods");
        colorSensor = map.colorSensor.get("colorsensor");

        leftHand = map.servo.get("left_hand");
        rightHand = map.servo.get("right_hand");

        touchBottom = map.touchSensor.get("touchbottom");
        touchTop = map.touchSensor.get("touchtop");

        cameraMonitorViewId = map.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", map.appContext.getPackageName());
    }

    public void retractHands() {
        leftHand.setPosition(IO.leftHandOut);
        rightHand.setPosition(IO.rightHandOut);
    }

    //public void setGyroCalibration() { gyro.calibrate(); }

    public void setGyroOffset() {
        gyroOffset = gyro.getHeading();
    }

    public void resetDriveEncoders() {
        leftOffset = leftDrive.getCurrentPosition();
        rightOffset = rightDrive.getCurrentPosition();
        forkLiftOffset = forkLiftMotor.getCurrentPosition();
        lastLeftEncoder = 0;
        lastRightEncoder = 0;
        lastForkLiftEncoder = 0;
        x = 0;
        y = 0;
    }
    public void updatePosition() {
        double leftEncoder = getLeftDriveEncoder();
        double rightEncoder = getRightDriveEncoder();
        double forkLiftEncoder = getForkLiftMotorEncoder();
        double averageChange = (leftEncoder - lastLeftEncoder + rightEncoder - lastRightEncoder)/2.0;
        double heading = Math.toRadians(getHeading());
        x += averageChange * Math.cos(heading);
        y += averageChange * Math.sin(heading);
        lastLeftEncoder = leftEncoder;
        lastRightEncoder = rightEncoder;
        lastForkLiftEncoder = forkLiftEncoder;
    }
    public double getX() {
        return x / COUNTSPERINCH;
    }
    public double getY() {
        return y / COUNTSPERINCH;
    }

    public double getHeading() {
        double heading = gyro.getHeading() - gyroOffset;
        while (heading > 180) {
            heading -= 360;
        }
        while (heading <= - 180) {
            heading += 360;
        }
        return -heading;
    }

    public void openGate() {
        //gate.setPosition(gateOpen);
    }
    public void closeGate() {
        //gate.setPosition(gateClosed);
    }

    public double getLeftDriveEncoder() {
        return leftDrive.getCurrentPosition() - leftOffset;
    }

    public double getRightDriveEncoder() {
        return rightDrive.getCurrentPosition() - rightOffset;
    }

    public double getForkLiftMotorEncoder() {
        return forkLiftMotor.getCurrentPosition() - forkLiftOffset;
    }

    public void setDrivePower(double left, double right) {
        leftDrive.setPower(left);
        rightDrive.setPower(right);
    }

    public double getLightReading(){
        return ods.getLightDetected();
    }

    public void calibrateGyro() {
        // start calibrating the gyro.
        telemetry.addData(">", "Gyro Calibrating. Do Not Move!");
        //telemetry.update();

        sleep(1000);

        //Thread.sleep(1000); // wait 1 second for gyro to stabilize (may be movement from initializing servo)
        gyro.calibrate();
    }

    public void setJewelColor(int jcolor){
        jewelColor = jcolor;
    }
    public int getJewelColor(){
        return jewelColor;
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
}
