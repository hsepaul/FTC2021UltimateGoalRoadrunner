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

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.utilities.*;

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

@TeleOp(name="Manual Driving OpMode", group="Iterative Opmode")
@Disabled
public class ManualDriving extends OpMode
{
    IO io;
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private boolean half_speed_engaged = false;
    private boolean half_speed_engaged_changed = false;
    static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position

    // Define class members
    double  position = 0; //(MAX_POS - MIN_POS) / 2; // Start at halfway position
    /*

     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        io = new IO(hardwareMap, telemetry);
        io.retractHands();
        telemetry.addData("Status", " Arms Initialized");
        //telemetry.addData("Calibrating Gyro", "Please Wait");

        //forkLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //forkLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //forkLiftMotor.setTargetPosition(forkLiftMotor.getCurrentPosition() + (int)(FORK_LIFT_TRAVEL * COUNTS_PER_INCH));
        //forkLiftMotor.setTargetPosition(2500);

        // Turn On RUN_TO_POSITION
        //forkLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("Fork Lift Encoder",  "Starting at %.2f",
                          io.getForkLiftMotorEncoder());

        telemetry.addData("Left Drive Encoder",  "Starting at %.2f",
                io.getLeftDriveEncoder());

        telemetry.addData("Right Drive Encoder",  "Starting at %.2f",
                io.getRightDriveEncoder());

        //io.calibrateGyro();
        //while (io.gyro.isCalibrating()){
            //wait for calibration to complete
        //}
        //telemetry.addData("Calibrating Gyro", "Complete");
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
        telemetry.addData("Status", "Resetting Encoders");
        runtime.reset();
        io.setGyroOffset();
        io.resetDriveEncoders();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;

        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        double drive = -gamepad1.right_stick_y;
        double turn  =  gamepad1.right_stick_x;
        leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
        rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;
        boolean upforkLift = gamepad2.y;
        boolean downforkLift = gamepad2.a;

        // Tank Mode uses one stick to control each wheel.
        // - This requires no math, but it is hard to drive forward slowly and keep straight.
        // leftPower  = -gamepad1.left_stick_y ;
        // rightPower = -gamepad1.right_stick_y ;

        // Send calculated power to wheels
        if(gamepad1.left_bumper && !half_speed_engaged_changed){
            half_speed_engaged = !half_speed_engaged;
            half_speed_engaged_changed = true;
        } else if (!gamepad1.left_bumper){
            half_speed_engaged_changed = false;
        }
        telemetry.addData("50% Power", half_speed_engaged);

        if (half_speed_engaged) {
            io.setDrivePower(leftPower/2, rightPower/2);
        } else {
            io.setDrivePower(leftPower, rightPower);
        }

        // Removed code when encoder motor for forklift replaced by Tetrix and extra push-button
        /*if((forkLiftMotor.getCurrentPosition() >= 5000) && (gamepad2.right_stick_y < 0)) {
            forkLiftMotor.setPower(0);
        } else if((forkLiftMotor.getCurrentPosition() <= 0) && (gamepad2.right_stick_y > 0)){
            forkLiftMotor.setPower(0);
        } else{
            forkLiftMotor.setPower(-gamepad2.right_stick_y);
        }*/

        if((io.touchTop.isPressed() == true) && (gamepad2.right_stick_y < 0)) {
            io.forkLiftMotor.setPower(0);
        } else if((io.touchBottom.isPressed() == true) && (gamepad2.right_stick_y > 0)){
            io.forkLiftMotor.setPower(0);
        } else{
            io.forkLiftMotor.setPower(-gamepad2.right_stick_y);
        }
        //} else {
        //    forkLiftMotor.setPower(0);
        //}


        //if(forkLiftMotor.getCurrentPosition() != (int)(FORK_LIFT_TRAVEL * COUNTS_PER_INCH)){
        //    forkLiftMotor.setPower(-gamepad2.right_stick_y);
        //} else {
        //    forkLiftMotor.setPower(0);
        //}

        io.arm.setPower(-gamepad2.left_stick_y);

        //leftHand.setPosition(.5);
        //rightHand.setPosition(.5);

        if (gamepad2.x) {
            // Keep stepping up until we hit the max value.
            position += INCREMENT ;
            if (position >= MAX_POS ) {
                position = MAX_POS;
                //rampUp = !rampUp;   // Switch ramp direction
            }
        }
        else if (gamepad2.b) {
            // Keep stepping down until we hit the min value.
            position -= INCREMENT ;
            if (position <= MIN_POS ) {
                position = MIN_POS;
                //rampUp = !rampUp;  // Switch ramp direction
            }
        }

        io.setHands(position);


        // Show the elapsed game time and wheel power.
        //telemetry.addData("Calibrating Gyro", "Complete");
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
        telemetry.addData("Fork Lift Encoder",  "Running at %.2f",
                io.getForkLiftMotorEncoder());
        telemetry.addData("Left Drive Encoder",  "Running at %.2f",
                io.getLeftDriveEncoder());
        telemetry.addData("Right Drive Encoder",  "Running at %.2f",
                io.getRightDriveEncoder());

        if (io.touchBottom.isPressed() == true) {
            telemetry.addData("Touch Bottom", "Is Pressed");
        } else {
            telemetry.addData("Touch Bottom", "Is Not Pressed");
        }

        if (io.touchTop.isPressed() == true) {
            telemetry.addData("Touch Top", "Is Pressed");
        } else {
            telemetry.addData("Touch Top", "Is Not Pressed");
        }

        telemetry.addData("IO Heading is", io.getHeading());
        telemetry.addData("Gyro Heading is", io.gyro.getHeading());
        telemetry.addData("Color is", io.colorSensor.toString());

        // convert the RGB values to HSV values.
        //Color.RGBToHSV(io.colorSensor.red() * 8, io.colorSensor.green() * 8, io.colorSensor.blue() * 8, hsvValues);

        // send the info back to driver station using telemetry function.
        telemetry.addData("LED", true ? "On" : "Off");
        telemetry.addData("Clear", io.colorSensor.alpha());
        telemetry.addData("Red  ", io.colorSensor.red());
        telemetry.addData("Green", io.colorSensor.green());
        telemetry.addData("Blue ", io.colorSensor.blue());
        //telemetry.addData("Hue", hsvValues[0]);

        // change the background color to match the color detected by the RGB sensor.
        // pass a reference to the hue, saturation, and value array as an argument
        // to the HSVToColor method.
        //relativeLayout.post(new Runnable() {
            //public void run() {
        //        relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
        //        //relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, hsvValues));
        //    }
        //});
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        // Removed code when encoder motor for forklift replaced by Tetrix and extra push-button
        /*runtime.reset();
        while ((touch.isPressed() == false) && (forkLiftMotor.getCurrentPosition() >= 0) && (runtime.seconds() <= .9)){
            forkLiftMotor.setPower(-1);
        }*/
        io.forkLiftMotor.setPower(0);
    }

}
