/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.utilities.IO_4WD_Test;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Calibration", group="Calibration")
@Disabled
public class Calibration extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    //private DcMotor forkLiftMotor = null;
    private DcMotor arm = null;
    IO_4WD_Test io;
    public Servo leftHand    = null;
    public Servo rightHand   = null;

    //public TouchSensor touch = null;  // Hardware Device Object

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 0.5 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    static final int        FORK_LIFT_TRAVEL        = 1;


    static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position

    // Define class members
    double  position = 0; //(MAX_POS - MIN_POS) / 2; // Start at halfway position
    boolean rampUp = true;
    boolean calibrationDone = false;
    boolean calibrationDoneLowerRelicArm = false;
    boolean calibrationDoneUpperRelicArm = false;
    long initTime = 0;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        io = new IO_4WD_Test(hardwareMap, telemetry);
        io.retractHands();
        io.openRelicHand();
        io.jewelArmUp();
        io.proximityArmUp();
        io.resetDriveEncoders();
        //io.resetDriveEncoders();
        //telemetry.addData("Status", "Resetting Encoders");
        telemetry.addData("Status", "Initialized");

/*        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        forkLiftMotor = hardwareMap.get(DcMotor.class, "fork_lift_motor");

        // get a reference to our digitalTouch object.
        touch = hardwareMap.get(TouchSensor.class, "touch");

        // set the digital channel to input.
        //touch.setMode(DigitalChannel.Mode.INPUT);

        arm = hardwareMap.get(DcMotor.class, "arm");

        leftHand  = hardwareMap.get(Servo.class, "left_hand");
        rightHand = hardwareMap.get(Servo.class, "right_hand");
        leftHand.setPosition(0);
        rightHand.setPosition(1);*/

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
/*        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        forkLiftMotor.setDirection(DcMotor.Direction.FORWARD);
        arm.setDirection(DcMotor.Direction.FORWARD);*/

        // Tell the driver that initialization is complete.
        if (io.touchBottom.getState() == false) {
            telemetry.addData("Calibration", "Not Required");
        } else {
            telemetry.addData("Calibration", "Required");
        }

        if (io.touchLowerRelicArm.getState() == false) {
            telemetry.addData("Calibration of Lower Relic Arm", "Not Required");
        } else {
            telemetry.addData("Calibration of Lower Relic Arm", "Required");
        }

        if (io.touchUpperRelicArm.getState() == false) {
            telemetry.addData("Calibration of Upper Relic Arm", "Not Required");
        } else {
            telemetry.addData("Calibration of Upper Relic Arm", "Required");
        }


        //forkLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //forkLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //forkLiftMotor.setTargetPosition(forkLiftMotor.getCurrentPosition() + (int)(FORK_LIFT_TRAVEL * COUNTS_PER_INCH));
        //forkLiftMotor.setTargetPosition(2500);

        // Turn On RUN_TO_POSITION
        //forkLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("Fork Lift",  "Setting at %.2f",
                          io.getForkLiftMotorEncoder());

        telemetry.addData("Lower Relic Arm",  "Setting at %.2f",
                io.getRPU1MotorEncoder());

        telemetry.addData("Upper Relic Arm",  "Setting at %.2f",
                io.getRPU2MotorEncoder());

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        initTime = System.currentTimeMillis();
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        //double leftPower;
        //double rightPower;

        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        //double drive = -gamepad1.left_stick_y;
        //double turn  =  gamepad1.right_stick_x;
        //leftPower    = Range.clip(drive - turn, -1.0, 1.0) ;
        //rightPower   = Range.clip(drive + turn, -1.0, 1.0) ;
        //boolean upforkLift = gamepad2.y;
        //boolean downforkLift = gamepad2.a;

        // Tank Mode uses one stick to control each wheel.
        // - This requires no math, but it is hard to drive forward slowly and keep straight.
        // leftPower  = -gamepad1.left_stick_y ;
        // rightPower = -gamepad1.right_stick_y ;

        // Send calculated power to wheels
        //leftDrive.setPower(leftPower);
        //rightDrive.setPower(rightPower);

        //if(upforkLift) {
        //    forkLiftMotor.setPower(1);
        //}

        //if(downforkLift) {
        //    forkLiftMotor.setPower(-1);
        //}



        //if((forkLiftMotor.getCurrentPosition() >= 5000) && (gamepad2.right_stick_y < 0)) {
        //    forkLiftMotor.setPower(0);
        //} else if((forkLiftMotor.getCurrentPosition() <= 0) && (gamepad2.right_stick_y > 0)){
        //    forkLiftMotor.setPower(0);
        //} else{
        //    forkLiftMotor.setPower(-gamepad2.right_stick_y);
        //}
        //} else {
        //    forkLiftMotor.setPower(0);
        //}


        //if(forkLiftMotor.getCurrentPosition() != (int)(FORK_LIFT_TRAVEL * COUNTS_PER_INCH)){
        //    forkLiftMotor.setPower(-gamepad2.right_stick_y);
        //} else {
        //    forkLiftMotor.setPower(0);
        //}

        //arm.setPower(-gamepad2.left_stick_y);

        //leftHand.setPosition(.5);
        //rightHand.setPosition(.5);

        //if (gamepad2.x) {
            // Keep stepping up until we hit the max value.
          //  position += INCREMENT ;
            //if (position >= MAX_POS ) {
              //  position = MAX_POS;
                //rampUp = !rampUp;   // Switch ramp direction
            //}
        //}
        //else if (gamepad2.b) {
            // Keep stepping down until we hit the min value.
          //  position -= INCREMENT ;
            //if (position <= MIN_POS ) {
              //  position = MIN_POS;
                //rampUp = !rampUp;  // Switch ramp direction
            //}
        //}

        //leftHand.setPosition(position);
        //rightHand.setPosition(1-position);

        if ((io.touchBottom.getState() == true) && (!calibrationDone) && ((System.currentTimeMillis() - initTime) > 0) && (System.currentTimeMillis() - initTime) <= 2000) {
            io.forkLiftMotor.setPower(.6);
            telemetry.addData("Calibration", "In Process");
        } else if ((io.touchBottom.getState() == true) && (!calibrationDone) && ((System.currentTimeMillis() - initTime) >= 2000) && (System.currentTimeMillis() - initTime) <= 4000) {
                    io.forkLiftMotor.setPower(-.6);
                    telemetry.addData("Calibration", "In Process");
        } else if (io.touchBottom.getState() == false){
            io.forkLiftMotor.setPower(0);
            io.resetDriveEncoders();
            telemetry.addData("Status", "Resetting Encoders");
            telemetry.addData("Calibration", "Completed");
            calibrationDone = true;
        }

        if ((io.touchLowerRelicArm.getState() == true) && (!calibrationDoneLowerRelicArm)){
            io.rpu1Motor.setPower(-.25);
            telemetry.addData("Calibration of Lower Relic Arm", "In Process");
        } else {
            io.rpu1Motor.setPower(0);
            io.resetDriveEncoders();
            telemetry.addData("Status", "Resetting Encoders");
            telemetry.addData("Calibration of Lower Relic Arm", "Completed");
            calibrationDoneLowerRelicArm = true;
        }

        if ((io.touchUpperRelicArm.getState() == true) && (!calibrationDoneUpperRelicArm)){
            io.rpu2Motor.setPower(-.25);
            telemetry.addData("Calibration of Upper Relic Arm", "In Process");
        } else {
            io.rpu2Motor.setPower(0);
            io.resetDriveEncoders();
            telemetry.addData("Status", "Resetting Encoders");
            telemetry.addData("Calibration of Upper Relic Arm", "Completed");
            calibrationDoneUpperRelicArm = true;
        }

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());

        if (io.touchBottom.getState() == false) {
            telemetry.addData("Touch Bottom", "Is Pressed");
        } else {
            telemetry.addData("Touch Bottom", "Is Not Pressed");
        }

        if (io.touchLowerRelicArm.getState() == false) {
            telemetry.addData("Touch Lower Relic Arm", "Is Pressed");
        } else {
            telemetry.addData("Touch Lower Relic Arm", "Is Not Pressed");
        }

        if (io.touchUpperRelicArm.getState() == false) {
            telemetry.addData("Touch Upper Relic Arm", "Is Pressed");
        } else {
            telemetry.addData("Touch Upper Relic Arm", "Is Not Pressed");
        }



        telemetry.addData("Fork Lift",  "Setting at %.2f",
                io.getForkLiftMotorEncoder());

        telemetry.addData("Lower Relic Arm",  "Setting at %.2f",
                io.getRPU1MotorEncoder());

        telemetry.addData("Upper Relic Arm",  "Setting at %.2f",
                io.getRPU2MotorEncoder());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
