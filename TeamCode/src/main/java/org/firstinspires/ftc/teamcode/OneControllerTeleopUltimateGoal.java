package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp

public class OneControllerTeleopUltimateGoal extends LinearOpMode {
//    private Gyroscope imu;
//    private DcMotor motorTest;
//    private DigitalChannel digitalTouch;
//    private DistanceSensor sensorColorRange;
//    private Servo servoTest;

    private DcMotor motorFL;
    private DcMotor motorFR;
    private DcMotor motorBL;
    private DcMotor motorBR;
    private DcMotor motorIN;
    private DcMotor motorAA;
    private DcMotor motorSH;
    private DcMotor motorSH1;
    private CRServo servoMAG;
    private Servo servoLL;
    private Servo servoLR;
    private Servo servoWG;
    private Servo servoLFT;

    //private boolean hookIsDown = false;
    //private boolean leftTriggerIsPressed = false;
    double interval = 0.05;
    double wobblePos = -.6;
    private double currentGripperRotatePosition;
    boolean magUp;
    boolean wobbleArmUp;
    @Override
    public void runOpMode() {
//        imu = hardwareMap.get(Gyroscope.class, "imu");
//        motorTest = hardwareMap.get(DcMotor.class, "motorTest");
//        digitalTouch = hardwareMap.get(DigitalChannel.class, "digitalTouch");
//        sensorColorRange = hardwareMap.get(DistanceSensor.class, "sensorColorRange");
//        servoTest = hardwareMap.get(Servo.class, "servoTest");

        // Define Motors & Servos
        motorFL = hardwareMap.get(DcMotor.class, "motorFL");
        motorFR = hardwareMap.get(DcMotor.class, "motorFR");
        motorBL = hardwareMap.get(DcMotor.class, "motorBL");
        motorBR = hardwareMap.get(DcMotor.class, "motorBR");
        motorIN = hardwareMap.get(DcMotor.class, "motorIN");
        motorAA = hardwareMap.get(DcMotor.class, "motorAA");
        motorSH = hardwareMap.get(DcMotor.class, "motorSH");
        motorSH1 = hardwareMap.get(DcMotor.class, "motorSH");
        servoMAG = hardwareMap.get(CRServo.class, "servoMAG");
        servoLL = hardwareMap.get(Servo.class, "servoLL");
        servoLR = hardwareMap.get(Servo.class, "servoLR");
        servoWG = hardwareMap.get(Servo.class, "servoWG");
        servoLFT = hardwareMap.get(Servo.class, "servoLFT");
        
        // Set Motor Power
        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);


        // Set Motor Mode
        motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // Set Right Motors to reverse values
        motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBR.setDirection(DcMotorSimple.Direction.REVERSE);


        // Set Motor zeroPowerBehavior
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set Servo Position
        servoWG.setPosition(1);
        servoLL.setPosition(.5);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.update();
            wheelCtrl();
        }
    }

    private void wheelCtrl() {

        //GAMEPAD 2
        double gamepad2LeftJoystickX = -gamepad2.left_stick_x; //
        double gamepad2LeftJoystickY = -gamepad2.left_stick_y; //

        double gamepad2RightJoystickX = -gamepad2.right_stick_x; //
        double gamepad2RightJoystickY = -gamepad2.right_stick_y; //

        //GAMEPAD 2

        //GAMEPAD 1
        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        double leftJoystickX = -gamepad1.left_stick_x; // Used to move robot left/right
        double leftJoystickY = -gamepad1.left_stick_y; // Used to move robot forward/backward

        double rightJoystickX = -gamepad1.right_stick_x; // Used to turn robot left(Counter-Clockwise)/right(Clockwise)
        double rightJoystickY = -gamepad1.right_stick_y; // Unused
        //GAMEPAD 1


//        if (gamepad2.left_trigger > 0) {
//            leftTriggerIsPressed = true;
//        } else if (gamepad2.left_trigger == 0) {
//            leftTriggerIsPressed = false;
//        }

//        if (gamepad1.y) {
//            if (hookIsDown) {
//                servoRH.setPosition(0.5);
//                servoLH.setPosition(0.5);
//                hookIsDown = false;
//            } else {
//                servoRH.setPosition(1);
//                servoLH.setPosition(1);
//                hookIsDown = true;
//            }
//        }



        //SENDING DATA TO PHONE WITH TELEMETRY
        telemetry.addData("Gamepad 2 Dpad Down Is Pressed", gamepad2.dpad_down);
        telemetry.addData("Left Joystick (X,Y)", "(" + leftJoystickX + "," + leftJoystickY + ")");
        telemetry.addData("Right Joystick (X,Y)", "(" + rightJoystickX + "," + rightJoystickY + ")");
        telemetry.addData("A button, servoLLposition", "(" + gamepad1.a + "," + servoLL.getPosition() + ")");
        //SENDING DATA TO PHONE WITH TELEMETRY

        //Triggers for Intake Control
        if (gamepad1.right_trigger > 0) {
            motorIN.setPower(1);
            if (!gamepad1.x){
                motorSH.setPower(-.75);
                motorSH1.setPower(-.75);
            }
        } else if (gamepad1.left_trigger > 0){
            motorIN.setPower(-1);
            if (!gamepad1.x){
                motorSH.setPower(.75);
                motorSH1.setPower(.75);
            }
        }
        else {
            motorIN.setPower(0);
        }
        //Dpad for Arm Angle
        if (gamepad1.dpad_down){
            motorAA.setPower(1);
        }
        else if (gamepad1.dpad_up){
            motorAA.setPower(-1);
        }
        else {
            motorAA.setPower(0);
        }
        //A button for feeder on
        if (gamepad1.a){
            servoMAG.setPower(-1);
        }
        else {
            servoMAG.setPower(0);
        }
        //X button for motors running
        if (gamepad1.x){
            motorSH.setPower(1);
            motorSH1.setPower(1);
        }
        //B and Y for rings up and down
        if (gamepad1.b){
            servoLFT.setPosition(-1);
        }
        if (gamepad1.y){
            servoLFT.setPosition(1);
        }
        //turn off shooter motors if intake and shooter are not being run
        if (!gamepad1.x && !(gamepad1.right_trigger > 0) && !(gamepad1.left_trigger > 0)){
            motorSH.setPower(0);
            motorSH1.setPower(0);
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
        // Setting the motors to a negative value will cause the robot to go forwards
        if (leftJoystickY < 0) { // Left Joystick Down (Backwards)
            telemetry.addData("Direction", "Down"); //GOING BACKWARDS, JOYSTICK DOWN
            motorFL.setPower(leftJoystickY); // -1
            motorFR.setPower(leftJoystickY); // -1
            motorBL.setPower(leftJoystickY); // -1
            motorBR.setPower(leftJoystickY); // -1
        } else if (leftJoystickY > 0) { // Left Joystick Up (Forwards)
            telemetry.addData("Direction", "Up"); //GOING FORWARDS WITH STICK UP
            motorFL.setPower(leftJoystickY); // 1
            motorFR.setPower(leftJoystickY); // 1
            motorBL.setPower(leftJoystickY); // 1
            motorBR.setPower(leftJoystickY); // 1
        } else if (leftJoystickX > 0) { // Left Joystick Left (Left)
            telemetry.addData("Direction", "Left"); //GOING LEFT WITH STICK SIDEWAYS LEFT
            motorFL.setPower(-leftJoystickX);  //  1
            motorFR.setPower(leftJoystickX);   // -1
            motorBL.setPower(leftJoystickX);   // -1
            motorBR.setPower(-leftJoystickX);  //  1
        } else if (leftJoystickX < 0) { // Left Joystick Right (Right)
            telemetry.addData("Direction", "Right"); //GOING RIGHT WITH STICK SIDEWAYS RIGHT
            motorFL.setPower(-leftJoystickX);  //  1
            motorFR.setPower(leftJoystickX);   // -1
            motorBL.setPower(leftJoystickX);   // -1
            motorBR.setPower(-leftJoystickX);  //  1
        } else {
            motorFL.setPower(0);
            motorFR.setPower(0);
            motorBL.setPower(0);
            motorBR.setPower(0);
        }


        // TURNING - TURNING - TURNING - TURNING - TURNING - TURNING - TURNING - //
        if (rightJoystickX > 0) { // Right Joystick Left (Counter Clockwise)
            telemetry.addData("Spin", "Counter Clockwise"); //TURNING COUNTER CLOCKWISE WITH STICK LEFT
            motorFL.setPower(-rightJoystickX); //  1
            motorFR.setPower(rightJoystickX);  // -1
            motorBL.setPower(-rightJoystickX); //  1
            motorBR.setPower(rightJoystickX);  // -1
        } else if (rightJoystickX < 0) { // Right Joystick Right (Clockwise)
            telemetry.addData("Spin", "Clockwise"); //TURNING CLOCKWISE WITH STICK RIGHT
            motorFL.setPower(-rightJoystickX); // -1
            motorFR.setPower(rightJoystickX);  //  1
            motorBL.setPower(-rightJoystickX); // -1
            motorBR.setPower(rightJoystickX);  //  1
        }
        // TURNING - TURNING - TURNING - TURNING - TURNING - TURNING - TURNING - //


        //ARM ANGLE CHANGED WITH RIGHT JOYSTICK



        //ARM EXTENDER CHANGED WITH DPAD



        //Gripper Wrist Position Set With X and Y position
        if (gamepad2LeftJoystickX > 0) { //joystick down
            currentGripperRotatePosition = 0.85;
        } else if (gamepad2LeftJoystickX < 0) { //joystick down
            currentGripperRotatePosition = 0.3;
        }
        //Gripper Wrist Position Set With X and Y position




        //Click 'B' and Gripper Wrist goes down about 0.7 further for more precision for being parallel to the ground/block.
        if (gamepad2.b) {
            currentGripperRotatePosition = 0.17;
        }

        if (gamepad2.y && gamepad2LeftJoystickY < 0) { //Change Gripper Wrist angle by 0.1 per refresh.
            currentGripperRotatePosition += interval;
            if (currentGripperRotatePosition >= 0.85) {
                currentGripperRotatePosition = 0.85;
            }
        }
        // Joystick down, Gripper Rotate Down
        if (gamepad2.y && gamepad2LeftJoystickY > 0) { //Change Gripper Wrist angle by 0.1 per refresh.
            currentGripperRotatePosition -= interval;
            if (currentGripperRotatePosition <= 0.15) {
                currentGripperRotatePosition = 0.15;
            }
        }


        //Gripper wrist Control



//        if (gamepad2RightJoystickX < 0.05 && gamepad2RightJoystickX > -0.05 && rightJoystickX > -0.05 && rightJoystickY < 0.05
//                && rightJoystickY > -0.05 && leftJoystickX > -0.05 && leftJoystickX < 0.05){
//            motorFL.setPower(0);
//            motorFR.setPower(0);
//            motorBL.setPower(0);
//            motorBR.setPower(0);
//            motorArmAngle.setPower(0);
//        }

        if (gamepad1.left_stick_button && gamepad1.right_stick_button){
            motorFL.setPower(0);
            motorFR.setPower(0);
            motorBL.setPower(0);
            motorBR.setPower(0);
        }
    }
    }
