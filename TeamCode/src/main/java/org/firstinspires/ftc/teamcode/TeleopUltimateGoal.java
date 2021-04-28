package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.ArmAngle;
import org.firstinspires.ftc.teamcode.commands.PoseStoring;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.advanced.PoseStorage;

import static java.lang.Math.round;

@TeleOp

public class TeleopUltimateGoal extends LinearOpMode {
//    private Gyroscope imu;
//    private DcMotor motorTest;
//    private DigitalChannel digitalTouch;
//    private DistanceSensor sensorColorRange;
//    private Servo servoTest;

    private DcMotorEx motorIN;
    private DcMotorEx motorAA;
    private DcMotorEx motorSH0;
    private DcMotorEx motorSH1;
    private Servo servoMAG;
    private Servo servoLL;
    private Servo servoLR;
    private Servo servoWG;
    private Servo servoLFT;
    private CRServo servoBRL;
    public SampleMecanumDrive drive;
    AnalogInput potentiometerAA;
    //magTimer
    private ElapsedTime magTimer;
    private ElapsedTime targetTimer;
    //private boolean hookIsDown = false;
    //private boolean leftTriggerIsPressed = false;
    double interval = 0.05;
    double wobblePos = -.6;
    private double currentGripperRotatePosition;
    public Pose2d currentPose;
    boolean magUp;
    boolean wobbleArmUp;
    double shooterAngle;
    double aimAngle;
    public int targetInt;
    public enum Target{
        highGoal,
        leftPowershot,
        middlePowershot,
        rightPowershot
    }
    public Target target;
    @Override
    public void runOpMode() {
//        imu = hardwareMap.get(Gyroscope.class, "imu");
//        motorTest = hardwareMap.get(DcMotor.class, "motorTest");
//        digitalTouch = hardwareMap.get(DigitalChannel.class, "digitalTouch");
//        sensorColorRange = hardwareMap.get(DistanceSensor.class, "sensorColorRange");
//        servoTest = hardwareMap.get(Servo.class, "servoTest");

        // Define Motors & Servos
        motorIN = hardwareMap.get(DcMotorEx.class, "motorIN");
        motorAA = hardwareMap.get(DcMotorEx.class, "motorAA");
        motorSH0 = hardwareMap.get(DcMotorEx.class, "motorSH0");
        motorSH1 = hardwareMap.get(DcMotorEx.class, "motorSH1");
        servoMAG = hardwareMap.get(Servo.class, "servoMAG");
        servoLL = hardwareMap.get(Servo.class, "servoLL");
        servoLR = hardwareMap.get(Servo.class, "servoLR");
        servoWG = hardwareMap.get(Servo.class, "servoWG");
        servoLFT = hardwareMap.get(Servo.class, "servoLFT");
        servoBRL = hardwareMap.get(CRServo.class, "servoBRL");
        potentiometerAA = hardwareMap.get(AnalogInput.class, "potentiometerAA");


        // Set Servo Position
        servoWG.setPosition(1);
        servoLL.setPosition(.5);
        servoMAG.setPosition(0);
        servoLFT.setPosition(1);
        aimAngle = 13;
        target = Target.highGoal;
        targetInt = 0;

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Pose", PoseStoring.autoPose);
        telemetry.update();
        //Init drive code
        drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        drive.setPoseEstimate(PoseStoring.autoPose);

        magTimer = new ElapsedTime();
        targetTimer = new ElapsedTime();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // run until the end of the match (driver presses STOP)
       while (opModeIsActive()) {
           drive.update();
           telemetry.addData("Status", "Running");
            telemetry.addData("Pose", drive.getPoseEstimate());
           telemetry.addData("shooter angle", shooterAngle);
           telemetry.update();
            wheelCtrl();
            currentPose = drive.getPoseEstimate();
           drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );



       }
    }

    public void wheelCtrl() {
        shooterAngle = round((10.0 * ((138.2488479 * potentiometerAA.getVoltage()) - 84.0829493))) / 10.0;

        //SENDING DATA TO PHONE WITH TELEMETRY
        telemetry.addData("DPAD DOWN", gamepad2.dpad_down);
        telemetry.addData("aim", target);
        //SENDING DATA TO PHONE WITH TELEMETRY

        //Triggers for Intake Control
        if (gamepad1.left_trigger > 0) {
            motorIN.setPower(1);
            servoBRL.setPower(-.6);
            if (!gamepad1.x){
                motorSH0.setVelocity(1200);
                motorSH1.setVelocity(-1200);
            }
        } else if (gamepad1.right_trigger > 0){
            motorIN.setPower(-1);
            servoBRL.setPower(.87);
            if (!gamepad1.x){
                motorSH0.setVelocity(-1200);
                motorSH1.setVelocity(1200);
            }
        }
        else {
            motorIN.setPower(0);
            servoBRL.setPower(0);
        }
        //Dpad for Arm Angle
        if (gamepad1.dpad_down){
            aimAngle = aimAngle - 1;
            ArmAngle.AngleArm(aimAngle, this);
        }
        else if (gamepad1.dpad_up){
            aimAngle = aimAngle + 1;
            ArmAngle.AngleArm(aimAngle, this);
        }
        else {
            motorAA.setPower(0);
        }
        //A button for feeder on
        if (magTimer.seconds() >= .1) {
            servoMAG.setPosition(.25);
            if (magTimer.seconds() >= .2) {
                if (gamepad1.a) {
                    servoMAG.setPosition(-1.3);
                    magTimer.reset();
                }
            }
        }

        //X button for motors running
        if (gamepad1.x){
            motorSH0.setVelocity(2000);
            motorSH1.setVelocity(-2000);
        }
        //dpad left and right for target setting
        if (targetTimer.seconds() > .1) {
            if (gamepad1.dpad_left && targetInt != 0) {
                targetInt -= 1;
                targetTimer.reset();
            }
            if (gamepad1.dpad_right && targetInt != 3) {
                targetInt += 1;
                targetTimer.reset();
            }
        }
        if (targetInt == 0){
            target = Target.highGoal;
        }
        if (targetInt == 1){
            target = Target.leftPowershot;
        }
        if (targetInt == 2){
            target = Target.middlePowershot;
        }
        if (targetInt == 3 ){
            target = Target.rightPowershot;
        }

        //B and Y shooter + rings up and down
        if (gamepad1.y){
            servoLFT.setPosition(.5);
            servoLL.setPosition(.8);
            if (target == Target.highGoal){
                Trajectory traj1 = drive.trajectoryBuilder(currentPose)
                        .lineToLinearHeading(new Pose2d(0, 34, Math.toRadians(0)))
                        .build();

                drive.followTrajectory(traj1);
                ArmAngle.AngleArm(25, this);
                aimAngle = 25;
            }
            if (target == Target.leftPowershot){
                Trajectory traj1 = drive.trajectoryBuilder(currentPose)
                        .lineToLinearHeading(new Pose2d(0, 18, Math.toRadians(0)))
                        .build();

                drive.followTrajectory(traj1);
                ArmAngle.AngleArm(23, this);
                aimAngle = 23;
            }
            if (target == Target.middlePowershot){
                Trajectory traj1 = drive.trajectoryBuilder(currentPose)
                        .lineToLinearHeading(new Pose2d(0, 10.5, Math.toRadians(0)))
                        .build();

                drive.followTrajectory(traj1);
                ArmAngle.AngleArm(23, this);
                aimAngle = 23;
            }
            if (target == Target.rightPowershot){
                Trajectory traj1 = drive.trajectoryBuilder(currentPose)
                        .lineToLinearHeading(new Pose2d(0, 3, Math.toRadians(0)))
                        .build();

                drive.followTrajectory(traj1);
                ArmAngle.AngleArm(23, this);
                aimAngle = 23;
            }
        }
        if(gamepad1.b){
            ArmAngle.AngleArm(13, this);
            servoLFT.setPosition(1);
        }
        //reset localizer
        if (gamepad1.back){
            drive.setPoseEstimate(new Pose2d(63, -14.5, 0));
        }
        //turn off shooter motors if intake and shooter are not being run
        if (!gamepad1.x && !(gamepad1.right_trigger > 0) && !(gamepad1.left_trigger > 0)){
            motorSH0.setVelocity(0);
            motorSH1.setVelocity(0);
        }
        //Wobble up and down and close
        if (gamepad1.right_bumper){
            servoLL.setPosition(0.1);
            servoLL.setPosition(0.05);
        }
        if (gamepad1.left_bumper){
            servoLL.setPosition(0.5);
        }
        if (gamepad1.right_stick_button){
            servoWG.setPosition(0);
        }
        if (gamepad1.left_stick_button){
            servoWG.setPosition(1);
        }
//        if (gamepad1.left_stick_button && gamepad1.right_stick_button){
//            motorFL.setPower(0);
//            motorFR.setPower(0);
//            motorBL.setPower(0);
//            motorBR.setPower(0);
//        }
    }
}
