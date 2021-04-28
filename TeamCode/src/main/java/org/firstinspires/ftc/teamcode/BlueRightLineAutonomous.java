package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.vision.UGContourRingPipeline;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.commands.ArmAngle;
import org.firstinspires.ftc.teamcode.commands.PoseStoring;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import static java.lang.Math.round;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;

@Autonomous

public class BlueRightLineAutonomous extends LinearOpMode {
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
    AnalogInput potentiometerAA;
    //magTimer
    private ElapsedTime magTimer;
    double shooterAngle;
    double aimAngle;
    int shootCount;
    int i;
    UGContourRingPipeline.Height ringCount;
    //FTCLIB camera
    private static final int CAMERA_WIDTH = 320; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 240; // height of wanted camera resolution

    private static final int HORIZON = 100; // horizon value to tune

    private static final boolean DEBUG = false; // if debug is wanted, change to true

    private static final boolean USING_WEBCAM = true; // change to true if using webcam
    private static final String WEBCAM_NAME = "frontWebcam"; // insert webcam name from configuration if using webcam

    private UGContourRingPipeline pipeline;
    private OpenCvCamera camera;

    public void runOpMode() {
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
        servoMAG.setPosition(0);
        servoLFT.setPosition(.5);

        aimAngle = 12;
        shootCount = 0;
        //Camera Code
        int cameraMonitorViewId = this
                .hardwareMap
                .appContext
                .getResources().getIdentifier(
                        "cameraMonitorViewId",
                        "id",
                        hardwareMap.appContext.getPackageName()
                );
        if (USING_WEBCAM) {
            camera = OpenCvCameraFactory
                    .getInstance()
                    .createWebcam(hardwareMap.get(WebcamName.class, WEBCAM_NAME), cameraMonitorViewId);
        } else {
            camera = OpenCvCameraFactory
                    .getInstance()
                    .createInternalCamera(
                            OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        }

        camera.setPipeline(pipeline = new UGContourRingPipeline(telemetry, DEBUG));

        UGContourRingPipeline.Config.setCAMERA_WIDTH(CAMERA_WIDTH);

        UGContourRingPipeline.Config.setHORIZON(HORIZON);

        camera.openCameraDeviceAsync(() -> camera.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT));

        //Get ring stack number with camera
        i = 0;
        ringCount = pipeline.getHeight();
        sleep(2500);
        while (i < 1000){
            ringCount = pipeline.getHeight();
            i = i + 1;
        }
        //Init drive code
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-62, 31, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        magTimer = new ElapsedTime();


        //Set trajectories ZERO RINGS
        Trajectory driveToRings = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(-34,34))
                .addTemporalMarker(.7, () ->{
                    motorSH0.setVelocity(2000);
                    motorSH1.setVelocity(-2000);
                })
                .build();

        Trajectory toWobbleZoneA1 = drive.trajectoryBuilder(driveToRings.end())
                .lineToLinearHeading(new Pose2d(-2,60, Math.toRadians(0)))
                .build();

        Trajectory toSecondWobbleFromA = drive.trajectoryBuilder(toWobbleZoneA1.end())
                .lineToLinearHeading(new Pose2d(-32,47, Math.toRadians(-180)))
                .build();

        Trajectory toWobbleZoneA2 = drive.trajectoryBuilder(toSecondWobbleFromA.end())
                .lineToLinearHeading(new Pose2d(-10,60, Math.toRadians(0)))
                .build();

        Trajectory parkFromA1 = drive.trajectoryBuilder(toWobbleZoneA2.end())
                .lineTo(new Vector2d(-20,36))
                .build();
        Trajectory parkFromA2 = drive.trajectoryBuilder(parkFromA1.end())
                .lineTo(new Vector2d(10,20))
                .build();
        Trajectory pushWobbles = drive.trajectoryBuilder(parkFromA2.end())
                .strafeLeft(24)
                .build();
        Trajectory antiPushWobbles = drive.trajectoryBuilder(pushWobbles.end())
                .strafeRight(12)
                .build();

        //Set trajectories ONE RING
        Trajectory pickUpOneRing = drive.trajectoryBuilder(driveToRings.end())
                .forward(10)
                .build();

        Trajectory toWobbleZoneB1 = drive.trajectoryBuilder(pickUpOneRing.end())
                .lineToLinearHeading(new Pose2d(22,34, Math.toRadians(0)))
                .build();

        Trajectory toSecondWobbleFromB = drive.trajectoryBuilder(toWobbleZoneB1.end())
                .lineToLinearHeading(new Pose2d(-32,47, Math.toRadians(180)))
                .build();

        Trajectory toWobbleZoneB2 = drive.trajectoryBuilder(toSecondWobbleFromB.end())
                .lineToLinearHeading(new Pose2d(15,40, Math.toRadians(0)))
                .build();

        //Set trajectories FOUR RINGS
        Trajectory pickUpThreeRings = drive.trajectoryBuilder(driveToRings.end())
                .forward(20)
                .build();
        Trajectory pickUpFourthRing = drive.trajectoryBuilder(pickUpThreeRings.end())
                .forward(10)
                .build();

        Trajectory aroundRings = drive.trajectoryBuilder(driveToRings.end())
                .strafeLeft(24)
                .build();
        Trajectory toWobbleZoneC1 = drive.trajectoryBuilder(pickUpFourthRing.end())
                .lineToLinearHeading(new Pose2d(46,56, Math.toRadians(0)))
                .build();

        Trajectory toSecondWobbleFromC = drive.trajectoryBuilder(toWobbleZoneC1.end())
                .lineToLinearHeading(new Pose2d(-32,47, Math.toRadians(180)))
                .build();

        Trajectory toWobbleZoneC2 = drive.trajectoryBuilder(toSecondWobbleFromC.end())
                .lineToLinearHeading(new Pose2d(38,58, Math.toRadians(0)))
                .build();

        Trajectory parkFromC = drive.trajectoryBuilder(toWobbleZoneC2.end())
                .back(36)
                .build();
        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Rings", ringCount);
        telemetry.update();
        waitForStart();
        if(isStopRequested()) return;
        servoLL.setPosition(.7);
        drive.followTrajectory(driveToRings);
        servoMAG.setPosition(.25);
        servoLFT.setPosition(.5);
        ArmAngle.AngleArm(24.7, this);
        while(ArmAngle.isFinished = false){

        }
        sleep(300);
        servoMAG.setPosition(-1.3);
        sleep(300);
        servoMAG.setPosition(.25);
        sleep(300);
        servoMAG.setPosition(-1.3);
        sleep(300);
        servoMAG.setPosition(.25);
        sleep(300);
        servoMAG.setPosition(-1.3);
        sleep(300);
        servoMAG.setPosition(.25);
        sleep(300);
        servoMAG.setPosition(-1.3);

        motorSH0.setVelocity(0);
        motorSH1.setVelocity(0);
        servoLFT.setPosition(1);
        ArmAngle.AngleArm(12, this);
        if (ringCount == UGContourRingPipeline.Height.ZERO){
            servoLL.setPosition(.4);
            drive.followTrajectory(toWobbleZoneA1);
            //drop wobble
            servoLL.setPosition(0);
            sleep(300);
            servoWG.setPosition(-1);
            sleep(300);
            drive.followTrajectory(toSecondWobbleFromA);

            //pick up wobble
            servoWG.setPosition(1);
            sleep(1100);
            servoLL.setPosition(.2);

            drive.followTrajectory(toWobbleZoneA2);
            //drop wobble
            servoLL.setPosition(0.1);
            sleep(300);
            servoWG.setPosition(-1);
            sleep(300);

            drive.followTrajectory(parkFromA1);
            drive.followTrajectory(parkFromA2);
//            drive.followTrajectory(pushWobbles);
//            drive.followTrajectory(antiPushWobbles);

            //prepare for tele
            ArmAngle.AngleArm(12, this);
        }
        if (ringCount == UGContourRingPipeline.Height.ONE){
            //pick up ring

            servoMAG.setPosition(.25);
            motorIN.setPower(-1);
            servoBRL.setPower(.85);
            motorSH0.setVelocity(-1200);
            motorSH1.setVelocity(1200);

            drive.followTrajectory(pickUpOneRing);
            //shoot ring
            sleep(4000);
            motorIN.setPower(0);
            servoBRL.setPower(0);

            servoLFT.setPosition(.5);
            motorSH0.setVelocity(2000);
            motorSH1.setVelocity(-2000);
            ArmAngle.AngleArm(24, this);
            servoMAG.setPosition(-1.3);
            sleep(150);
            servoMAG.setPosition(.25);
            sleep(150);
            servoMAG.setPosition(-1.3);
            motorSH0.setVelocity(-0);
            motorSH1.setVelocity(0);
            drive.followTrajectory(toWobbleZoneB1);

            //drop wobble
            servoLL.setPosition(0.1);
            sleep(300);
            servoWG.setPosition(-1);
            sleep(300);

            drive.followTrajectory(toSecondWobbleFromB);

            //pick up wobble
            servoWG.setPosition(1);
            sleep(1100);
            servoLL.setPosition(.2);

            drive.followTrajectory(toWobbleZoneB2);

            //drop wobble
            servoLL.setPosition(0.1);
            sleep(300);
            servoWG.setPosition(-1);

            //prepare for tele
            ArmAngle.AngleArm(12, this);
        }
        if (ringCount == UGContourRingPipeline.Height.FOUR){
            drive.followTrajectory(aroundRings);

            drive.followTrajectory(toWobbleZoneC1);
            servoLL.setPosition(0);
            sleep(300);
            servoWG.setPosition(-1);
            sleep(300);
            drive.followTrajectory(toSecondWobbleFromC);
            servoWG.setPosition(1);
            sleep(1100);
            servoLL.setPosition(.2);

            drive.followTrajectory(toWobbleZoneC2);
            servoLL.setPosition(0);
            sleep(300);
            servoWG.setPosition(-1);
            sleep(300);
            sleep(500);
            drive.followTrajectory(parkFromC);
            //prepare for tele
            ArmAngle.AngleArm(12, this);
        }
        else{
            telemetry.addData("Ring Count Failed", "Rip your OPR");
            telemetry.update();
        }
        PoseStoring.autoPose = drive.getPoseEstimate();
        telemetry.addData("Pose", PoseStoring.autoPose);
        telemetry.update();
    }
}
