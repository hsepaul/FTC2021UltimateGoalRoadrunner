package org.firstinspires.ftc.teamcode;
// import lines were omitted. OnBotJava will add them automatically.
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp
public class JavaEncoderTest extends LinearOpMode {
    DcMotorEx motorSH1;
    DcMotorEx motorAA;
    DcMotorEx motorIN;
    DcMotorEx motorFL;
    DcMotorEx motorFR;
    DcMotorEx motorBL;
    DcMotorEx motorBR;
    
    @Override
    public void runOpMode() {
        motorFL = hardwareMap.get(DcMotorEx.class, "motorFL");
        motorFR = hardwareMap.get(DcMotorEx.class, "motorFR");
        motorBL = hardwareMap.get(DcMotorEx.class, "motorBL");
        motorBR = hardwareMap.get(DcMotorEx.class, "motorBR");
        motorIN = hardwareMap.get(DcMotorEx.class, "motorIN");
        motorAA = hardwareMap.get(DcMotorEx.class, "motorAA");
        motorSH1 = hardwareMap.get(DcMotorEx.class, "motorSH1");
        
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
        waitForStart();
        
        while (opModeIsActive()) {
            telemetry.addData("Right F/B encoder value", motorSH1.getCurrentPosition());
            telemetry.addData("Left F/B encoder value", motorIN.getCurrentPosition());
            telemetry.addData("L/R encoder value", motorAA.getCurrentPosition());
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

        if (gamepad1.left_stick_button && gamepad1.right_stick_button){
            motorFL.setPower(0);
            motorFR.setPower(0);
            motorBL.setPower(0);
            motorBR.setPower(0);
        }
    }
}
