package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp

public class UltimateGoalTeleopRed extends LinearOpMode {
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
    private DcMotor motorSH0;
    private DcMotor motorSH1;
    private DcMotor motorAA;
    private Servo servoWG;
    private Servo servoLL;
    private Servo servoLFT;
    private CRServo servoMAG;
   
   // private Servo servoGP;
    //private boolean hookIsDown = false;
    //private boolean leftTriggerIsPressed = false;
    private boolean wgCLOSED = true;
    private double wobbleLiftServoPOS = .5;
    double interval = 0.05;
 

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
        
        motorSH0 = hardwareMap.get(DcMotor.class, "motorSH0");
        motorSH1 = hardwareMap.get(DcMotor.class, "motorSH1");
        motorAA = hardwareMap.get(DcMotor.class, "motorAA");
        
        servoMAG = hardwareMap.get(CRServo.class, "servoMAG");
        
        servoWG = hardwareMap.get(Servo.class, "servoWG");
        servoLFT = hardwareMap.get(Servo.class, "servoLFT");
        servoLL = hardwareMap.get(Servo.class, "servoLL");

        // Set Motor Power
        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);
        motorIN.setPower(0);
        motorSH0.setPower(0);
        motorSH1.setPower(0);
        motorAA.setPower(0);

        // Set Motor Mode
        motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorIN.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorSH0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorSH1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorAA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        // Set Right Motors to reverse values
        motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBR.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set Motor zeroPowerBehavior
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorIN.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorSH0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorSH1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorAA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        
        // Set Servo Position
        servoWG.setPosition(1);
        servoLFT.setPosition(0);
        servoLL.setPosition(wobbleLiftServoPOS);

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

        
        //ALWAYS HAVE THIS
        servoLL.setPosition(wobbleLiftServoPOS);
        //Always HAVE THIS
        
        // SLOW MODE BELOW, THEN RAW DATA
        
         if 	(gamepad1.left_trigger < 0 && leftJoystickY < 0) { // Left Joystick Down (Backwards)
            telemetry.addData("Direction", "Down"); //GOING BACKWARDS, JOYSTICK DOWN
            motorFL.setPower(-0.4); // -1
            motorFR.setPower(-0.4); // -1
            motorBL.setPower(-0.4); // -1
            motorBR.setPower(-0.4); // -1
        } else if (gamepad1.left_trigger > 0 && leftJoystickY > 0) { // Left Joystick Up (Forwards)
            telemetry.addData("Direction", "Up"); //GOING FORWARDS WITH STICK UP
            motorFL.setPower(0.4); // 1
            motorFR.setPower(0.4); // 1
            motorBL.setPower(0.4); // 1
            motorBR.setPower(0.4); // 1
        } else if (gamepad1.left_trigger > 0 && leftJoystickX > 0) { // Left Joystick Left (Left)
            telemetry.addData("Direction", "Left"); //GOING LEFT WITH STICK SIDEWAYS LEFT
            motorFL.setPower(0.4);  //  1
            motorFR.setPower(-0.4);   // -1
            motorBL.setPower(-0.4);   // -1
            motorBR.setPower(0.4);  //  1
        } else if (gamepad1.left_trigger < 0 && leftJoystickX < 0) { // Left Joystick Right (Right)
            telemetry.addData("Direction", "Right"); //GOING RIGHT WITH STICK SIDEWAYS RIGHT
            motorFL.setPower(0.4);  //  1
            motorFR.setPower(-0.4);   // -1
            motorBL.setPower(-0.4);   // -1
            motorBR.setPower(0.4);  //  1
        } else if (leftJoystickY < 0) { // Left Joystick Down (Backwards)
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
        if (gamepad1.left_trigger > 0 && rightJoystickX > 0) { // Right Joystick Left (Counter Clockwise)
            telemetry.addData("Spin", "Counter Clockwise"); //TURNING COUNTER CLOCKWISE WITH STICK LEFT
            motorFL.setPower(0.4); //  1
            motorFR.setPower(-0.4);  // -1   SLOW MODE
            motorBL.setPower(0.4); //  1
            motorBR.setPower(-0.4);  // -1
        } else if (gamepad1.left_trigger < 0 && rightJoystickX < 0) { // Right Joystick Right (Clockwise)
            telemetry.addData("Spin", "Clockwise"); //TURNING CLOCKWISE WITH STICK RIGHT
            motorFL.setPower(-0.4); // -1
            motorFR.setPower(0.4);  //  1
            motorBL.setPower(-0.4); // -1
            motorBR.setPower(0.4);  //  1
        } else if (rightJoystickX > 0) { // Right Joystick Left (Counter Clockwise)
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

        // INTAKE
             if (gamepad2.left_trigger > 0) {
            motorIN.setPower(1);
            motorSH0.setPower(0.5);
            motorSH1.setPower(-0.5);
        } else if (gamepad2.right_trigger > 0) {
            motorIN.setPower(-1);
            motorSH0.setPower(-0.5);
            motorSH1.setPower(0.5);
        } else {
            motorIN.setPower(0);
        }
        
        
        //WG open close
        
        if (gamepad2.left_stick_button && wgCLOSED) {
            servoWG.setPosition(0.5);
            wgCLOSED = false;
        } else if (gamepad2.left_stick_button && !wgCLOSED) {
            servoWG.setPosition(0);
            wgCLOSED = true;
          }
        
        
        // Right joystick up/down to for wobble lift
        
        if (gamepad2RightJoystickY < 0 && wobbleLiftServoPOS >= 0) { //Joystick going down
            //wobbleLiftServoPOS -= interval;
            wobbleLiftServoPOS = .9;
        }
        
        if (gamepad2RightJoystickY > 0 && wobbleLiftServoPOS < 0.8) { //Joystick going UP
            //wobbleLiftServoPOS += interval;
            wobbleLiftServoPOS = .1;
        }
        
        
        
        
        // Servo MAG, SHoots one ring
        if (gamepad2.a) {
            servoMAG.setPower(1); 
        } else {
            servoMAG.setPower(0);
        }
        
        
        if (gamepad2.b) {
            motorAA.setPower(1);
        } else if (gamepad2.x) { 
            motorAA.setPower(-1);
            motorSH0.setPower(0);
            motorSH1.setPower(0);
        } else {
            motorAA.setPower(0);
            motorSH0.setPower(0);
            motorSH1.setPower(0);
        }
                              
        if (gamepad2.y) {
            motorSH0.setPower(0.75);
            motorSH1.setPower(-0.75);
        }
           
        if (gamepad1.left_stick_button && gamepad1.right_stick_button){
            motorFL.setPower(0);
            motorFR.setPower(0);
            motorBL.setPower(0);
            motorBR.setPower(0);
            motorIN.setPower(0);
            motorSH0.setPower(0);
            motorSH1.setPower(0);
            motorAA.setPower(0);            
        }
        
    }
    }
