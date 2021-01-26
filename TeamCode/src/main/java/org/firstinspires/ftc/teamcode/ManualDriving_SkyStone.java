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
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.commands.ArmAngleMovement;
import org.firstinspires.ftc.teamcode.commands.BasicCommand;
//import org.firstinspires.ftc.teamcode.commands.DOM2Movement;
import org.firstinspires.ftc.teamcode.utilities.IO_SkyStone_Test;

import java.util.ArrayList;
import java.util.Iterator;

//import org.firstinspires.ftc.teamcode.utilities.IO;
//import org.firstinspires.ftc.teamcode.utilities.IO_4WD_Test;
//import org.firstinspires.ftc.teamcode.utilities.PID;

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

@TeleOp(name="Manual Driving Sky Stone OpMode", group="Iterative Opmode")
//@Disabled
public class ManualDriving_SkyStone extends OpMode
{
    IO_SkyStone_Test io;
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    //private boolean half_speed_engaged = false;
    //private boolean half_speed_engaged_changed = false;

    private boolean relic_retrieval_engaged = false;
    private boolean relic_retrieval_engaged_changed = false;

    private boolean relic_retrieval_stow_engaged = false;
    private boolean relic_retrieval_stow_engaged_changed = false;

    private boolean relic_retrieval_score_engaged = false;
    private boolean relic_retrieval_score_engaged_changed = false;

    private boolean full_speed_engaged = false;
    private boolean low_speed_engaged = false;
    static final double INCREMENT   = 0.025;     // amount to slew servo each CYCLE_MS cycle
    static final double INCREMENTSLOW   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position

    // Define class members
    double  position = 1; //(MAX_POS - MIN_POS) / 2; // Start at halfway position
    double  position2 = 1; //(MAX_POS - MIN_POS) / 2; // Start at halfway position

    //double positionCapstone = .9;

    int skystone_level = 1;
    boolean skystone_level_changed_up = false;
    boolean skystone_level_changed_down = false;
    boolean gripper_rotate_stowed_changed = false;
    boolean gripper_rotate_stowed_commanded = false;

    boolean gripper_rotate_stowed_locked = false;

    double initStartingPositionArmAngle;

    //double relicRetrievalStartingPositionRPU1;
    //double relicRetrievalStartingPositionRPU2;
    //double relicScoreStartingPositionRPU1;
    //double relicScoreStartingPositionRPU2;

    static final int INIT = 0;
    static final int EXECUTE = 1;
    static final int STOP = 2;
    static final int FINISHED = 3;

    //int state;
    //int initStateRPU1;
    //int initStateRPU2;
    //int retrieveRelicStateRPU1;
    //int retrieveRelicStateRPU2;
    //int retrieveRelicStowStateRPU1;
    //int retrieveRelicStowStateRPU2;
    //int retrieveRelicScoreStateRPU1;
    //int retrieveRelicScoreStateRPU2;
    //ArrayList<BasicCommand> commands;
    ArrayList<BasicCommand> commandsInitArmAngle;
    //ArrayList<BasicCommand> commandsRetrieveRelicRPU1;
    //ArrayList<BasicCommand> commandsRetrieveRelicRPU2;
    //ArrayList<BasicCommand> commandsRetrieveRelicStowRPU1;
    //ArrayList<BasicCommand> commandsRetrieveRelicStowRPU2;
    //ArrayList<BasicCommand> commandsRetrieveRelicScoreRPU1;
    //ArrayList<BasicCommand> commandsRetrieveRelicScoreRPU2;
    //BasicCommand currentCommand;
    BasicCommand currentCommandInitArmAngle;
    //BasicCommand currentCommandRetrieveRelicRPU1;
    //BasicCommand currentCommandRetrieveRelicRPU2;
    //BasicCommand currentCommandRetrieveRelicStowRPU1;
    //BasicCommand currentCommandRetrieveRelicStowRPU2;
    //BasicCommand currentCommandRetrieveRelicScoreRPU1;
    //BasicCommand currentCommandRetrieveRelicScoreRPU2;
    //Iterator<BasicCommand> iterator;
    Iterator<BasicCommand> iteratorInitArmAngle;
    //Iterator<BasicCommand> iteratorRetrieveRelicRPU1;
    //Iterator<BasicCommand> iteratorRetrieveRelicRPU2;
    //Iterator<BasicCommand> iteratorRetrieveRelicStowRPU1;
    //Iterator<BasicCommand> iteratorRetrieveRelicStowRPU2;
    //Iterator<BasicCommand> iteratorRetrieveRelicScoreRPU1;
    //Iterator<BasicCommand> iteratorRetrieveRelicScoreRPU2;

    /*

     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        io = new IO_SkyStone_Test(hardwareMap, telemetry);
        //io.retractHands();
        //io.openRelicHand();
        //io.jewelArmUp();
        ///io.proximityArmMid();
        //io.gripperRotateStowed();
        io.gripperPincherStopped();
        io.gripperPincher2Stopped();
        //io.leftHookMid();
        //io.rightHookMid();
        io.rightHookMidDown();
        io.leftHookUp();
        //io.rightHookUp();
        io.capStoneUp();
        io.resetDriveEncoders();

        BasicCommand.setIO(io);
        BasicCommand.setTelemetry(telemetry);

        //commands = new ArrayList<BasicCommand>();
        commandsInitArmAngle = new ArrayList<BasicCommand>();
        //commandsRetrieveRelicRPU1 = new ArrayList<BasicCommand>();
        //commandsRetrieveRelicRPU2 = new ArrayList<BasicCommand>();
        //commandsRetrieveRelicStowRPU1 = new ArrayList<BasicCommand>();
        //commandsRetrieveRelicStowRPU2 = new ArrayList<BasicCommand>();
        //commandsRetrieveRelicScoreRPU1 = new ArrayList<BasicCommand>();
        //commandsRetrieveRelicScoreRPU2 = new ArrayList<BasicCommand>();
        addInitArmAngleCommands();
        //addInitDOM2Commands();
        //addRetrieveRelicRPU1Commands();
        //addRetrieveRelicRPU2Commands();
        //addRetrieveRelicStowRPU1Commands();
        //addRetrieveRelicStowRPU2Commands();
        //addRetrieveRelicScoreRPU1Commands();
        //addRetrieveRelicScoreRPU2Commands();
        //addCommands();
        //addFinalCommands();
        //iterator = commands.iterator();
        iteratorInitArmAngle = commandsInitArmAngle.iterator();
        //iteratorInitDOM2 = commandsInitDOM2.iterator();
        //iteratorRetrieveRelicRPU1 = commandsRetrieveRelicRPU1.iterator();
        //iteratorRetrieveRelicRPU2 = commandsRetrieveRelicRPU2.iterator();
        //iteratorRetrieveRelicStowRPU1 = commandsRetrieveRelicStowRPU1.iterator();
        //iteratorRetrieveRelicStowRPU2 = commandsRetrieveRelicStowRPU2.iterator();
        //iteratorRetrieveRelicScoreRPU1 = commandsRetrieveRelicScoreRPU1.iterator();
        //iteratorRetrieveRelicScoreRPU2 = commandsRetrieveRelicScoreRPU2.iterator();
        //currentCommand = iterator.next();
        currentCommandInitArmAngle = iteratorInitArmAngle.next();
        //currentCommandInitDOM2 = iteratorInitDOM2.next();
        //currentCommandRetrieveRelicRPU1 = iteratorRetrieveRelicRPU1.next();
        //currentCommandRetrieveRelicRPU2 = iteratorRetrieveRelicRPU2.next();
        //currentCommandRetrieveRelicStowRPU1 = iteratorRetrieveRelicStowRPU1.next();
        //currentCommandRetrieveRelicStowRPU2 = iteratorRetrieveRelicStowRPU2.next();
        //currentCommandRetrieveRelicScoreRPU1 = iteratorRetrieveRelicScoreRPU1.next();
        //currentCommandRetrieveRelicScoreRPU2 = iteratorRetrieveRelicScoreRPU2.next();

        initStartingPositionArmAngle = ((ArmAngleMovement) currentCommandInitArmAngle).targetPosition;
        //initStartingPositionDOM2 = ((DOM2Movement) currentCommandInitDOM2).targetPosition;
        //relicRetrievalStartingPositionRPU1 = ((RPU1Movement) currentCommandRetrieveRelicRPU1).targetPosition;
        //relicRetrievalStartingPositionRPU2 = ((RPU2Movement) currentCommandRetrieveRelicRPU2).targetPosition;
        //relicScoreStartingPositionRPU1 = ((RPU1Movement) currentCommandRetrieveRelicScoreRPU1).targetPosition;
        //relicScoreStartingPositionRPU2 = ((RPU2Movement) currentCommandRetrieveRelicScoreRPU2).targetPosition;
        //state = INIT;
        //initStateRPU1 = INIT;
        //initStateRPU2 = INIT;
        //retrieveRelicStateRPU1 = INIT;
        //retrieveRelicStateRPU2 = INIT;
        //retrieveRelicStowStateRPU1 = INIT;
        //retrieveRelicStowStateRPU2 = INIT;
        //retrieveRelicScoreStateRPU1 = INIT;
        //retrieveRelicScoreStateRPU2 = INIT;



/*        telemetry.addData("Right Back Drive Encoder",  "Starting at %.2f",
                io.getRightBackDriveEncoder());
        telemetry.addData("Left Back Drive Encoder",  "Starting at %.2f",
                io.getLeftBackDriveEncoder());
        telemetry.addData("Right Front Drive Encoder",  "Starting at %.2f",
                io.getRightFrontDriveEncoder());
        telemetry.addData("Left Front Drive Encoder",  "Starting at %.2f",
                io.getLeftFrontDriveEncoder());*/
        io.calibrateGyroandIMU();
        //io.calibrateGyroandIMU1();
        telemetry.addData("4WD!", "Go");
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
        io.resetDriveEncoders();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        io.updatePosition(); //For RevBulkData
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;

        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        //double drive = -gamepad1.right_stick_y; original
        //double turn  =  gamepad1.right_stick_x; original
        //leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
        //rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;




        //double cosA = Math.cos(Math.toRadians(io.getIMUHeading()));
        //double sinA = Math.sin(Math.toRadians(io.getIMUHeading()));

        if (Math.abs(gamepad1.left_stick_x) <.25) {
            gamepad1.left_stick_x = 0;
        }

        if (Math.abs(gamepad1.left_stick_y) <.25) {
            gamepad1.left_stick_y = 0;
        }



        double cosA = Math.cos(Math.toRadians(0.0));
        double sinA = Math.sin(Math.toRadians(0.0));
        double x1 = -gamepad1.left_stick_x*cosA - gamepad1.left_stick_y*sinA;
        double y1 = -gamepad1.left_stick_x*sinA + gamepad1.left_stick_y*cosA;
        //double rotation = -gamepad1.right_stick_x * .6;
        double rotation = -gamepad1.right_stick_x;


        double[] wheelPowers = new double[4];
        wheelPowers[0] = x1 + y1 + rotation;   //FL
        wheelPowers[1] = -x1 + y1 - rotation;  //FR
        wheelPowers[2] = -x1 + y1 + rotation;  //BL
        wheelPowers[3] = x1 + y1 - rotation;   //BR
        io.normalize(wheelPowers);

        double wheelPowerFL = wheelPowers[0];
        double wheelPowerFR = wheelPowers[1];
        double wheelPowerBL = wheelPowers[2];
        double wheelPowerBR = wheelPowers[3];

        //double drive =  gamepad1.right_stick_y;
        //double turn  = -gamepad1.right_stick_x;
        //leftPower    = Range.clip(drive - turn, -1.0, 1.0) ;
        //rightPower   = Range.clip(drive + turn, -1.0, 1.0) ;


        // Tank Mode uses one stick to control each wheel.
        // - This requires no math, but it is hard to drive forward slowly and keep straight.
        // leftPower  = -gamepad1.left_stick_y ;
        // rightPower = -gamepad1.right_stick_y ;

        // Send calculated power to wheels
        if(gamepad1.left_bumper){
            full_speed_engaged = true;
        } else if (!gamepad1.left_bumper){
            full_speed_engaged = false;
        }

        if(gamepad1.left_trigger > 0){
            low_speed_engaged = true;
        } else if (gamepad1.left_trigger == 0){
            low_speed_engaged = false;
        }


        //code for a single push turn on of half speed. must press-release-press to engage.
/*        if(gamepad1.left_bumper && !half_speed_engaged_changed){
            half_speed_engaged = !half_speed_engaged;
            half_speed_engaged_changed = true;
        } else if (!gamepad1.left_bumper){
            half_speed_engaged_changed = false;
        }*/
        telemetry.addData("100% Power", full_speed_engaged);
        telemetry.addData("60% Power", (!low_speed_engaged && !full_speed_engaged));
        telemetry.addData("<60% Power", low_speed_engaged);
        if (full_speed_engaged){
            telemetry.addData("Power", "%d%%",100);
        } else {
            telemetry.addData("Power", "%.2f%%",((.6 - (.4*gamepad1.left_trigger))*100));
        }


        if (full_speed_engaged) {
            io.setDrivePower(wheelPowerFL, wheelPowerFR, wheelPowerBL, wheelPowerBR);
        } else if (low_speed_engaged) {
                io.setDrivePower(((.6 - (.4*gamepad1.left_trigger))*wheelPowerFL), ((.6 - (.4*gamepad1.left_trigger))*wheelPowerFR), ((.6 - (.4*gamepad1.left_trigger))*wheelPowerBL), ((.6 - (.4*gamepad1.left_trigger))*wheelPowerBR));
        } else {
            io.setDrivePower((wheelPowerFL*.6), (wheelPowerFR*.6), (wheelPowerBL*.6), (wheelPowerBR*.6));
        }











        if((io.getArmExtenderEncoder() <= -2800) && (gamepad2.dpad_up)) {
            io.armExtenderMotor.setPower(0);
        }
        else if ((io.getArmExtenderEncoder() <= -2500) && (gamepad2.dpad_up)){
            io.armExtenderMotor.setPower((.75)*-1);
        }
        else if(((io.getArmExtenderEncoder() >= 0) || (io.touchArmExtender.getState() == false)) && (gamepad2.dpad_down)){
            io.armExtenderMotor.setPower(0);
        }
        else if (gamepad2.dpad_up){
            if (low_speed_engaged) {
                io.armExtenderMotor.setPower(((.6 - (.4 * gamepad1.left_trigger)) * 1)*-1);
            } else {
                io.armExtenderMotor.setPower(-1);
            }
        }
        else if (gamepad2.dpad_down){
            if (low_speed_engaged) {
                io.armExtenderMotor.setPower(((.6 - (.4 * gamepad1.left_trigger))));
            } else {
                io.armExtenderMotor.setPower(1);
            }
        }
        else {
            io.armExtenderMotor.setPower(0);
        }


        /*if((io.getForkLiftMotorEncoder() <= -4000) && (gamepad2.left_stick_y < 0)) {
            io.forkLiftMotor.setPower(0);
        } else if(((io.getForkLiftMotorEncoder() >= 0) || (io.touchBottom.getState() == false)) && (gamepad2.left_stick_y > 0)){
            io.forkLiftMotor.setPower(0);
        } else{
            io.forkLiftMotor.setPower(gamepad2.left_stick_y);
        }
*/
        //io.rpu1Motor.setPower(.4*gamepad2.right_stick_y);
        //io.rpu2Motor.setPower(.4*gamepad2.right_stick_x);

/*        if((io.touchTop.getState() == false) && (gamepad2.left_stick_y < 0)) {
            io.forkLiftMotor.setPower(0);
        } else if((io.touchBottom.getState() == false) && (gamepad2.left_stick_y > 0)){
            io.forkLiftMotor.setPower(0);
        } else{
            io.forkLiftMotor.setPower(-gamepad2.left_stick_y);
        }*/

/*        if((io.touchBottom.getState() == false) && (gamepad2.left_stick_y > 0)){
            io.forkLiftMotor.setPower(0);
        } else{
            io.forkLiftMotor.setPower(gamepad2.left_stick_y);
        }*/

        /*if (gamepad2.x) {
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

        if (gamepad2.y) {
            // Keep stepping up until we hit the max value.
            position2 += INCREMENT ;
            if (position2 >= MAX_POS ) {
                position2 = MAX_POS;
                //rampUp = !rampUp;   // Switch ramp direction
            }
        }
        else if (gamepad2.a) {
            // Keep stepping down until we hit the min value.
            position2 -= INCREMENT ;
            if (position2 <= MIN_POS ) {
                position2 = MIN_POS;
                //rampUp = !rampUp;  // Switch ramp direction
            }
        }*/

/*        if (gamepad1.x) {
            io.hookCounterClockwise();
        } else if (gamepad1.b){
            io.hookClockwise();
        } else {
            io.hookStop();
        }

        if (gamepad2.x) {
            io.domSweepMotor.setPower(1);
        } else if (gamepad2.b){
            io.domSweepMotor.setPower(-1);
        } else {
            io.domSweepMotor.setPower(0);
        }*/


        if (gamepad1.x) {
            //io.rightHookUp();
            io.leftHookUp();
            //io.rightHookMid();
            io.rightHookMidDown();
            //io.leftHookMid();
        }

        if (gamepad1.b) {
            io.rightHookDown();
            io.leftHookDown();
        }

        if (gamepad2.x) {
            io.capStoneUp();
            //positionCapstone = .9;
        }

        if (gamepad2.a) {
            io.capStoneDown();

            /*positionCapstone -= INCREMENTSLOW;
            if (positionCapstone <= MIN_POS) {
                positionCapstone = MIN_POS;
                //rampUp = !rampUp;  // Switch ramp direction
            }*/

            //io.capStone.setPosition(positionCapstone);
        }

        if (gamepad2.left_trigger > 0) {
            //io.gripperRotateStowed();
            //position = 1;
            position = .9;
        }

        /*if (gripper_rotate_stowed_locked) {
            //io.gripperRotateStowed();
            //position = 1;
            position = .9;
        }*/

        /*if (gamepad2.y) {
            io.gripperRotateParallel();
            position = .30;
        }*/

        //if (gamepad2.a) {
        //    io.gripperRotateDown();
        //}

        if (gamepad2.left_bumper) {
            //io.gripperPincherOpen();
            io.gripperPincherOpenSlow();
            //io.gripperPincher2Open();
            io.gripperPincher2OpenSlow();
            if (io.getArmAngleEncoder() <= -8000){
                io.armAngleMotor.setPower(0);
            } else {
                io.armAngleMotor.setPower(-1);
            }
        } else if (gamepad2.right_bumper) {
            io.gripperPincherClosed();
            io.gripperPincher2Closed();
        } else {
            io.gripperPincherStopped();
            io.gripperPincher2Stopped();
        }


        if (!gripper_rotate_stowed_locked && !gripper_rotate_stowed_commanded) {
            if (gamepad2.left_stick_y > 0) {
                // Keep stepping up until we hit the max value.
                position += INCREMENT;
                if (position >= MAX_POS) {
                    position = MAX_POS;
                    //rampUp = !rampUp;   // Switch ramp direction
                }
            } else if (gamepad2.left_stick_y < 0) {
                // Keep stepping down until we hit the min value.
                position -= INCREMENT;
                if (position <= MIN_POS) {
                    position = MIN_POS;
                    //rampUp = !rampUp;  // Switch ramp direction
                }
            }
        }


        /*if (gamepad2.left_bumper) {
            io.markerBoxUp();
        }

        if (gamepad2.right_bumper) {
            io.markerBoxDown();
        }

        if (gamepad1.right_bumper) {
            io.markerBoxFlat();
        }*/


        /*telemetry.addData("Elevator Hand Position", position);
        io.setHands(position);
        telemetry.addData("Relic Hand Position", position2);
        io.setRelicHand(position2);*/
        //io.setJewelArm(position);

        //stateMachine(initState, currentCommandInit, iteratorInit);

        /*if(gamepad2.right_bumper && !relic_retrieval_engaged_changed){
            relic_retrieval_engaged = !relic_retrieval_engaged;
            relic_retrieval_stow_engaged = false;
            relic_retrieval_score_engaged = false;
            relic_retrieval_engaged_changed = true;
        } else if (!gamepad2.right_bumper) {
            relic_retrieval_engaged_changed = false;
        }

        if(gamepad2.left_bumper && !relic_retrieval_stow_engaged_changed && relic_retrieval_engaged){
            relic_retrieval_stow_engaged = !relic_retrieval_stow_engaged;
            relic_retrieval_engaged = false;
            relic_retrieval_score_engaged = false;
            relic_retrieval_stow_engaged_changed = true;
        } else if (!gamepad2.left_bumper) {
            relic_retrieval_stow_engaged_changed = false;
        }

        if((gamepad2.right_trigger == 1) && !relic_retrieval_score_engaged_changed && relic_retrieval_stow_engaged){
            relic_retrieval_score_engaged = !relic_retrieval_score_engaged;
            relic_retrieval_engaged = false;
            relic_retrieval_stow_engaged = false;
            relic_retrieval_score_engaged_changed = true;
        } else if (gamepad2.right_trigger != 1) {
            relic_retrieval_score_engaged_changed = false;
        }*/


        if(gamepad2.left_trigger > 0 && !skystone_level_changed_up){
            skystone_level += 1;
            skystone_level_changed_up = true;
            if (skystone_level >= 6 ) {
                skystone_level = 6;
            }
        } else if (gamepad2.left_trigger == 0 ) {
            skystone_level_changed_up = false;
        }

        if(gamepad2.right_trigger > 0 && !skystone_level_changed_down){
            skystone_level -= 1;
            skystone_level_changed_down = true;
            if (skystone_level <= 1 ) {
                skystone_level = 1;
            }
        } else if (gamepad2.right_trigger == 0) {
            skystone_level_changed_down = false;
        }





        /*currentCommandInitArmAngle.execute();

        if (skystone_level == 1) {
            initStartingPositionArmAngle = 0;
            if ((initStartingPositionArmAngle >= 240) || (io.touchArmAngle.getState() == false) ) {
                initStartingPositionArmAngle = 240;
                //rampUp = !rampUp;   // Switch ramp direction
            }
        }*/





        if((io.getArmAngleEncoder() <= -8000) && (gamepad2.right_stick_y > 0)) {
            io.armAngleMotor.setPower(0);
        }
        else if ((io.getArmAngleEncoder() <= -7500) && (gamepad2.right_stick_y > 0)){
            io.armAngleMotor.setPower((.5)*-1);
        }
        else if(((io.getArmAngleEncoder() >= 0) || (io.touchArmAngle.getState() == false)) && (gamepad2.right_stick_y < 0)){
            io.armAngleMotor.setPower(0);
        }
        else if (gamepad2.right_stick_y > 0){
            if (low_speed_engaged) {
                io.armAngleMotor.setPower(((.6 - (.4 * gamepad1.left_trigger)) * 1)*-1);
            } else {
                io.armAngleMotor.setPower(-1);
            }
        }
        else if (gamepad2.right_stick_y < 0){
            if (low_speed_engaged) {
                io.armAngleMotor.setPower(((.6 - (.4 * gamepad1.left_trigger))));
            } else {
                io.armAngleMotor.setPower(1);
            }
        }
        else {
            if (!gamepad2.left_bumper){
                io.armAngleMotor.setPower(0);
            }
            //io.armAngleMotor.setPower(0);
        }




        if(gamepad2.left_trigger > 0 && !gripper_rotate_stowed_changed){
            gripper_rotate_stowed_changed = true;
            gripper_rotate_stowed_commanded = !gripper_rotate_stowed_commanded;
        } else if (gamepad2.left_trigger == 0) {
            gripper_rotate_stowed_changed = false;
        }

        /*if (gamepad2.b && !gamepad2.start) {
            gripper_rotate_stowed_locked = false;
        }*/

        if (!gamepad2.y && !gripper_rotate_stowed_commanded && !gripper_rotate_stowed_locked) {
            //keep gripper parallel
            position = (io.getArmAngleEncoder() + 10779) / 30961;
            //position = (io.getArmAngleEncoder() + 11960) / 35321;
            //position = (io.getArmAngleEncoder() + 8503.6) / 23883;
            //position = (io.getArmAngleEncoder() + 9223) / 29018;
            //position = (io.getArmAngleEncoder() + 7363.3) / 36000;
            //position = (io.getArmAngleEncoder() + 7772.1) / 29018;
            //position = (io.getArmAngleEncoder() + 8100.1) / 27821;
            //position = (io.getArmAngleEncoder() + 7896.4) / 28053;
        }

        io.gripperRotate.setPosition(position);





        /*currentCommandInitArmAngle.execute();
        //currentCommandInitDOM2.execute();

        if (gamepad2.left_stick_y == 1) {
            // Keep stepping up until we hit the max value.
            initStartingPositionArmAngle += 7;
            if ((initStartingPositionArmAngle >= 240) || (io.touchArmAngle.getState() == false) ) {
                initStartingPositionArmAngle = 240;
                //rampUp = !rampUp;   // Switch ramp direction
            }
        }
        else if (gamepad2.left_stick_y < 1 && gamepad2.left_stick_y >= .5){
            initStartingPositionArmAngle += 5;
            if ((initStartingPositionArmAngle >= 240) || (io.touchArmAngle.getState() == false) ) {
                initStartingPositionArmAngle = 240;
                //rampUp = !rampUp;   // Switch ramp direction
            }
        }
        else if (gamepad2.left_stick_y < .5 && gamepad2.left_stick_y >= .03){
            initStartingPositionArmAngle += 2;
            if ((initStartingPositionArmAngle >= 240) || (io.touchArmAngle.getState() == false) ) {
                initStartingPositionArmAngle = 240;
                //rampUp = !rampUp;   // Switch ramp direction
            }
        }
        else if (gamepad2.left_stick_y == -1) {
            // Keep stepping down until we hit the min value.
            initStartingPositionArmAngle -= 7;
            if (initStartingPositionArmAngle <= 0) {
                initStartingPositionArmAngle = 0;
                //rampUp = !rampUp;  // Switch ramp direction
            }
        } else if (gamepad2.left_stick_y > -1 && gamepad2.left_stick_y <= -.5){
            initStartingPositionArmAngle -= 5;
            if (initStartingPositionArmAngle <= 0) {
                initStartingPositionArmAngle = 0;
                //rampUp = !rampUp;  // Switch ramp direction
            }
        }
        else if (gamepad2.left_stick_y > -.5 && gamepad2.left_stick_y <= -.03){
            initStartingPositionArmAngle -= 2;
            if (initStartingPositionArmAngle <= 0) {
                initStartingPositionArmAngle = 0;
                //rampUp = !rampUp;  // Switch ramp direction
            }
        }*/



        /*if((io.getDOMMotorExtendEncoder() >= 9450) && (gamepad2.right_stick_x > 0)) {
            io.domExtendMotor.setPower(0);*/

        /*if((io.touchDOMExtend.getState() == false) && (gamepad2.right_stick_x < 0)) {
            io.domExtendMotor.setPower(0);
        } else {
            io.domExtendMotor.setPower(gamepad2.right_stick_x);
        }*/


        /*if (gamepad2.right_stick_x == 1) {
            // Keep stepping up until we hit the max value.
            relicRetrievalStartingPositionRPU2 += 10 ;
            if (relicRetrievalStartingPositionRPU2 >= 800 ) {
                relicRetrievalStartingPositionRPU2 = 800;
                //rampUp = !rampUp;   // Switch ramp direction
            }
        }
        else if (gamepad2.right_stick_x == -1) {
            // Keep stepping down until we hit the min value.
            relicRetrievalStartingPositionRPU2 -= 10 ;
            if (relicRetrievalStartingPositionRPU2 <= 0 ) {
                relicRetrievalStartingPositionRPU2 = 0;
                //rampUp = !rampUp;  // Switch ramp direction
            }
        }*/

        ((ArmAngleMovement) currentCommandInitArmAngle).armAnglePID.setTarget(initStartingPositionArmAngle);

        /*if (!relic_retrieval_engaged && !relic_retrieval_stow_engaged && !relic_retrieval_score_engaged) {

*//*            telemetry.addData("RPU1 Target Position: ", initStartingPositionRPU1);
            telemetry.addData("RPU2 Target Position: ", initStartingPositionRPU2);*//*

            currentCommandInitRPU1.execute();
            currentCommandInitRPU2.execute();
        }*/

        /*if (relic_retrieval_engaged && !relic_retrieval_stow_engaged && !relic_retrieval_score_engaged) {

*//*            telemetry.addData("RPU1 Target Position: ", relicRetrievalStartingPositionRPU1);
            telemetry.addData("RPU2 Target Position: ", relicRetrievalStartingPositionRPU2);*//*

            currentCommandRetrieveRelicRPU1.execute();
            currentCommandRetrieveRelicRPU2.execute();

            if (gamepad2.right_stick_y == 1) {
                // Keep stepping up until we hit the max value.
                relicRetrievalStartingPositionRPU1 += 10 ;
                if (relicRetrievalStartingPositionRPU1 >= 800 ) {
                    relicRetrievalStartingPositionRPU1 = 800;
                    //rampUp = !rampUp;   // Switch ramp direction
                }
            }
            else if (gamepad2.right_stick_y == -1) {
                // Keep stepping down until we hit the min value.
                relicRetrievalStartingPositionRPU1 -= 10 ;
                if (relicRetrievalStartingPositionRPU1 <= 0 ) {
                    relicRetrievalStartingPositionRPU1 = 0;
                    //rampUp = !rampUp;  // Switch ramp direction
                }
            }

            if (gamepad2.right_stick_x == 1) {
                // Keep stepping up until we hit the max value.
                relicRetrievalStartingPositionRPU2 += 10 ;
                if (relicRetrievalStartingPositionRPU2 >= 800 ) {
                    relicRetrievalStartingPositionRPU2 = 800;
                    //rampUp = !rampUp;   // Switch ramp direction
                }
            }
            else if (gamepad2.right_stick_x == -1) {
                // Keep stepping down until we hit the min value.
                relicRetrievalStartingPositionRPU2 -= 10 ;
                if (relicRetrievalStartingPositionRPU2 <= 0 ) {
                    relicRetrievalStartingPositionRPU2 = 0;
                    //rampUp = !rampUp;  // Switch ramp direction
                }
            }

            ((RPU1Movement) currentCommandRetrieveRelicRPU1).rpu1PID.setTarget(relicRetrievalStartingPositionRPU1);
            ((RPU2Movement) currentCommandRetrieveRelicRPU2).rpu2PID.setTarget(relicRetrievalStartingPositionRPU2);
        }*/

        /*if (relic_retrieval_stow_engaged && !relic_retrieval_engaged && !relic_retrieval_score_engaged) {
            currentCommandRetrieveRelicStowRPU1.execute();
            currentCommandRetrieveRelicStowRPU2.execute();
        }*/

        /*if (relic_retrieval_score_engaged && !relic_retrieval_stow_engaged && !relic_retrieval_engaged) {

*//*            telemetry.addData("RPU1 Target Position: ", relicScoreStartingPositionRPU1);
            telemetry.addData("RPU2 Target Position: ", relicScoreStartingPositionRPU2);*//*

            currentCommandRetrieveRelicScoreRPU1.execute();
            currentCommandRetrieveRelicScoreRPU2.execute();

            if (gamepad2.right_stick_y == 1) {
                // Keep stepping up until we hit the max value.
                relicScoreStartingPositionRPU1 += 10 ;
                if (relicScoreStartingPositionRPU1 >= 800 ) {
                    relicScoreStartingPositionRPU1 = 800;
                    //rampUp = !rampUp;   // Switch ramp direction
                }
            }
            else if (gamepad2.right_stick_y == -1) {
                // Keep stepping down until we hit the min value.
                relicScoreStartingPositionRPU1 -= 10 ;
                if (relicScoreStartingPositionRPU1 <= 0 ) {
                    relicScoreStartingPositionRPU1 = 0;
                    //rampUp = !rampUp;  // Switch ramp direction
                }
            }

            if (gamepad2.right_stick_x == 1) {
                // Keep stepping up until we hit the max value.
                relicScoreStartingPositionRPU2 += 10 ;
                if (relicScoreStartingPositionRPU2 >= 800 ) {
                    relicScoreStartingPositionRPU2 = 800;
                    //rampUp = !rampUp;   // Switch ramp direction
                }
            }
            else if (gamepad2.right_stick_x == -1) {
                // Keep stepping down until we hit the min value.
                relicScoreStartingPositionRPU2 -= 10 ;
                if (relicScoreStartingPositionRPU2 <= 0 ) {
                    relicScoreStartingPositionRPU2 = 0;
                    //rampUp = !rampUp;  // Switch ramp direction
                }
            }

            ((RPU1Movement) currentCommandRetrieveRelicScoreRPU1).rpu1PID.setTarget(relicScoreStartingPositionRPU1);
            ((RPU2Movement) currentCommandRetrieveRelicScoreRPU2).rpu2PID.setTarget(relicScoreStartingPositionRPU2);
        }*/


        /*if (!relic_retrieval_engaged && !relic_retrieval_stow_engaged && !relic_retrieval_score_engaged) {

            switch (initStateRPU1) {
                case INIT:
                    currentCommandInitRPU1.init();
                    initStateRPU1 = EXECUTE;
                    break;
                case EXECUTE:
                    if (currentCommandInitRPU1.isFinished()) {
                        currentCommandInitRPU1.stop();
                        if (iteratorInitRPU1.hasNext()) {
                            currentCommandInitRPU1 = iteratorInitRPU1.next();
                            initStateRPU1 = INIT;
                        } else initStateRPU1 = EXECUTE; //was FINISHED
                        break;
                    }
                    currentCommandInitRPU1.execute();
                    break;
                case STOP:
                    currentCommandInitRPU1.stop();
                    if (iteratorInitRPU1.hasNext()) {
                        currentCommandInitRPU1 = iteratorInitRPU1.next();
                        initStateRPU1 = INIT;
                    } else initStateRPU1 = FINISHED;
                    break;
                case FINISHED:
                    break;
            }

            switch (initStateRPU2) {
                case INIT:
                    currentCommandInitRPU2.init();
                    initStateRPU2 = EXECUTE;
                    break;
                case EXECUTE:
                    if (currentCommandInitRPU2.isFinished()) {
                        currentCommandInitRPU2.stop();
                        if (iteratorInitRPU2.hasNext()) {
                            currentCommandInitRPU2 = iteratorInitRPU2.next();
                            initStateRPU2 = INIT;
                        } else initStateRPU2 = EXECUTE; //was FINISHED
                        break;
                    }
                    currentCommandInitRPU2.execute();
                    break;
                case STOP:
                    currentCommandInitRPU2.stop();
                    if (iteratorInitRPU2.hasNext()) {
                        currentCommandInitRPU2 = iteratorInitRPU2.next();
                        initStateRPU2 = INIT;
                    } else initStateRPU2 = FINISHED;
                    break;
                case FINISHED:
                    break;
            }
        }

        if (relic_retrieval_engaged && !relic_retrieval_stow_engaged && !relic_retrieval_score_engaged) {

            switch (retrieveRelicStateRPU1) {
                case INIT:
                    currentCommandRetrieveRelicRPU1.init();
                    retrieveRelicStateRPU1 = EXECUTE;
                    break;
                case EXECUTE:
                    if (currentCommandRetrieveRelicRPU1.isFinished()) {
                        currentCommandRetrieveRelicRPU1.stop();
                        if (iteratorRetrieveRelicRPU1.hasNext()) {
                            currentCommandRetrieveRelicRPU1 = iteratorRetrieveRelicRPU1.next();
                            retrieveRelicStateRPU1 = INIT;
                        } else retrieveRelicStateRPU1 = EXECUTE; //was FINISHED
                        break;
                    }
                    currentCommandRetrieveRelicRPU1.execute();
                    break;
                case STOP:
                    currentCommandRetrieveRelicRPU1.stop();
                    if (iteratorRetrieveRelicRPU1.hasNext()) {
                        currentCommandRetrieveRelicRPU1 = iteratorRetrieveRelicRPU1.next();
                        retrieveRelicStateRPU1 = INIT;
                    } else retrieveRelicStateRPU1 = FINISHED;
                    break;
                case FINISHED:
                    break;
            }

            switch (retrieveRelicStateRPU2) {
                case INIT:
                    currentCommandRetrieveRelicRPU2.init();
                    retrieveRelicStateRPU2 = EXECUTE;
                    break;
                case EXECUTE:
                    if (currentCommandRetrieveRelicRPU2.isFinished()) {
                        currentCommandRetrieveRelicRPU2.stop();
                        if (iteratorRetrieveRelicRPU2.hasNext()) {
                            currentCommandRetrieveRelicRPU2 = iteratorRetrieveRelicRPU2.next();
                            retrieveRelicStateRPU2= INIT;
                        } else retrieveRelicStateRPU2 = EXECUTE; //was FINISHED
                        break;
                    }
                    currentCommandRetrieveRelicRPU2.execute();
                    break;
                case STOP:
                    currentCommandRetrieveRelicRPU2.stop();
                    if (iteratorRetrieveRelicRPU2.hasNext()) {
                        currentCommandRetrieveRelicRPU2 = iteratorRetrieveRelicRPU2.next();
                        retrieveRelicStateRPU2 = INIT;
                    } else retrieveRelicStateRPU2 = FINISHED;
                    break;
                case FINISHED:
                    break;
            }


        }

        if (relic_retrieval_stow_engaged && !relic_retrieval_engaged && !relic_retrieval_score_engaged) {

            switch (retrieveRelicStowStateRPU1) {
                case INIT:
                    currentCommandRetrieveRelicStowRPU1.init();
                    retrieveRelicStowStateRPU1 = EXECUTE;
                    break;
                case EXECUTE:
                    if (currentCommandRetrieveRelicStowRPU1.isFinished()) {
                        currentCommandRetrieveRelicStowRPU1.stop();
                        if (iteratorRetrieveRelicStowRPU1.hasNext()) {
                            currentCommandRetrieveRelicStowRPU1 = iteratorRetrieveRelicStowRPU1.next();
                            retrieveRelicStowStateRPU1 = INIT;
                        } else retrieveRelicStowStateRPU1 = EXECUTE; //was FINISHED
                        break;
                    }
                    currentCommandRetrieveRelicStowRPU1.execute();
                    break;
                case STOP:
                    currentCommandRetrieveRelicStowRPU1.stop();
                    if (iteratorRetrieveRelicStowRPU1.hasNext()) {
                        currentCommandRetrieveRelicStowRPU1 = iteratorRetrieveRelicStowRPU1.next();
                        retrieveRelicStowStateRPU1 = INIT;
                    } else retrieveRelicStowStateRPU1 = FINISHED;
                    break;
                case FINISHED:
                    break;
            }

            switch (retrieveRelicStowStateRPU2) {
                case INIT:
                    currentCommandRetrieveRelicStowRPU2.init();
                    retrieveRelicStowStateRPU2 = EXECUTE;
                    break;
                case EXECUTE:
                    if (currentCommandRetrieveRelicStowRPU2.isFinished()) {
                        currentCommandRetrieveRelicStowRPU2.stop();
                        if (iteratorRetrieveRelicStowRPU2.hasNext()) {
                            currentCommandRetrieveRelicStowRPU2 = iteratorRetrieveRelicStowRPU2.next();
                            retrieveRelicStowStateRPU2 = INIT;
                        } else retrieveRelicStowStateRPU2 = EXECUTE; //was FINISHED
                        break;
                    }
                    currentCommandRetrieveRelicStowRPU2.execute();
                    break;
                case STOP:
                    currentCommandRetrieveRelicStowRPU2.stop();
                    if (iteratorRetrieveRelicStowRPU2.hasNext()) {
                        currentCommandRetrieveRelicStowRPU2 = iteratorRetrieveRelicStowRPU2.next();
                        retrieveRelicStowStateRPU2 = INIT;
                    } else retrieveRelicStowStateRPU2 = FINISHED;
                    break;
                case FINISHED:
                    break;
            }

        }

        if (relic_retrieval_score_engaged && !relic_retrieval_stow_engaged && !relic_retrieval_engaged) {

            switch (retrieveRelicScoreStateRPU1) {
                case INIT:
                    currentCommandRetrieveRelicScoreRPU1.init();
                    retrieveRelicScoreStateRPU1 = EXECUTE;
                    break;
                case EXECUTE:
                    if (currentCommandRetrieveRelicScoreRPU1.isFinished()) {
                        currentCommandRetrieveRelicScoreRPU1.stop();
                        if (iteratorRetrieveRelicScoreRPU1.hasNext()) {
                            currentCommandRetrieveRelicScoreRPU1 = iteratorRetrieveRelicScoreRPU1.next();
                            retrieveRelicScoreStateRPU1 = INIT;
                        } else retrieveRelicScoreStateRPU1 = EXECUTE; //was FINISHED
                        break;
                    }
                    currentCommandRetrieveRelicScoreRPU1.execute();
                    break;
                case STOP:
                    currentCommandRetrieveRelicScoreRPU1.stop();
                    if (iteratorRetrieveRelicScoreRPU1.hasNext()) {
                        currentCommandRetrieveRelicScoreRPU1 = iteratorRetrieveRelicScoreRPU1.next();
                        retrieveRelicScoreStateRPU1 = INIT;
                    } else retrieveRelicScoreStateRPU1 = FINISHED;
                    break;
                case FINISHED:
                    break;
            }

            switch (retrieveRelicScoreStateRPU2) {
                case INIT:
                    currentCommandRetrieveRelicScoreRPU2.init();
                    retrieveRelicScoreStateRPU2 = EXECUTE;
                    break;
                case EXECUTE:
                    if (currentCommandRetrieveRelicScoreRPU2.isFinished()) {
                        currentCommandRetrieveRelicScoreRPU2.stop();
                        if (iteratorRetrieveRelicScoreRPU2.hasNext()) {
                            currentCommandRetrieveRelicScoreRPU2 = iteratorRetrieveRelicScoreRPU2.next();
                            retrieveRelicScoreStateRPU2 = INIT;
                        } else retrieveRelicScoreStateRPU2 = EXECUTE; //was FINISHED
                        break;
                    }
                    currentCommandRetrieveRelicScoreRPU2.execute();
                    break;
                case STOP:
                    currentCommandRetrieveRelicScoreRPU2.stop();
                    if (iteratorRetrieveRelicScoreRPU2.hasNext()) {
                        currentCommandRetrieveRelicScoreRPU2 = iteratorRetrieveRelicScoreRPU2.next();
                        retrieveRelicScoreStateRPU2 = INIT;
                    } else retrieveRelicScoreStateRPU2 = FINISHED;
                    break;
                case FINISHED:
                    break;
            }


        }*/






/*        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        //telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
        //telemetry.addData("GamePad2 Left Stick Y", "left stick (%.2f)", gamepad2.left_stick_y);

        if (!relic_retrieval_engaged && !relic_retrieval_stow_engaged && !relic_retrieval_score_engaged) {
            telemetry.addData("Relic", "Init");
        } else {
            telemetry.addData("Relic", "Not Init");*/
/*        }
        telemetry.addData("relic", "relic_retrieval_engaged: " + relic_retrieval_engaged);
        telemetry.addData("relic", "relic_retrieval_stow_engaged: " + relic_retrieval_stow_engaged);
        telemetry.addData("relic", "relic_retrieval_score_engaged: " + relic_retrieval_score_engaged);

        telemetry.addData("Fork Lift Encoder",  "Running at %.2f",
                io.getForkLiftMotorEncoder());

        telemetry.addData("RPU1 Drive Encoder",  "Starting at %.2f",
                io.getRPU1MotorEncoder());

        telemetry.addData("RPU2 Drive Encoder",  "Starting at %.2f",
                io.getRPU2MotorEncoder());

        telemetry.addData("Right Back Drive Encoder",  "Starting at %.2f",
                io.getRightBackDriveEncoder());
        telemetry.addData("Left Back Drive Encoder",  "Starting at %.2f",
                io.getLeftBackDriveEncoder());
        telemetry.addData("Right Front Drive Encoder",  "Starting at %.2f",
                io.getRightFrontDriveEncoder());
        telemetry.addData("Left Front Drive Encoder",  "Starting at %.2f",
                io.getLeftFrontDriveEncoder());*/

        /*telemetry.addData("Odometer Left Encoder",  "Starting at %.2f",
                io.getOdometerLeftEncoder());
        telemetry.addData("Odometer Center Encoder",  "Starting at %.2f",
                io.getOdometerCenterEncoder());
        telemetry.addData("Odometer Right Encoder",  "Starting at %.2f",
                io.getOdometerRightEncoder());
        telemetry.addData("Arm Extender Encoder",  "Starting at %.2f",
                io.getArmExtenderEncoder());
        telemetry.addData("Arm Angle Encoder",  "Starting at %.2f",
                io.getArmAngleEncoder());*/

        /*telemetry.addData("DOM1 Motor Encoder",  "Starting at %.2f",
                io.getDOM1MotorEncoder());
        telemetry.addData("DOM2 Motor Encoder",  "Starting at %.2f",
                io.getDOM2MotorEncoder());
        telemetry.addData("DOM Motor Extend Encoder",  "Starting at %.2f",
                io.getDOMMotorExtendEncoder());*/

        /*telemetry.addData("Left Front Range", String.format("%.01f in", io.leftFrontDistance.getDistance(DistanceUnit.INCH)));
        telemetry.addData("Right Front Range", String.format("%.01f in", io.rightFrontDistance.getDistance(DistanceUnit.INCH)));
        //telemetry.addData("Front Range", String.format("%.01f in", io.frontDistance.getDistance(DistanceUnit.INCH)));
        //telemetry.addData("Back Range", String.format("%.01f in", io.backDistance.getDistance(DistanceUnit.INCH)));

        telemetry.addData("Potentiometer Target Arm Angle", String.format("%.01f degrees", initStartingPositionArmAngle));
        telemetry.addData("Potentiometer Arm Angle Voltage", String.format("%.01f volts", io.getArmAnglePotVoltage()));
        telemetry.addData("Potentiometer Arm Angle Degrees", String.format("%.01f degrees", (io.getArmAnglePotDegrees())));
        telemetry.addData("Gripper Rotate Servo Position", String.format("%.2f servo increments", position));
        telemetry.addData("Sky Stone Level", skystone_level);


        telemetry.addData("FL Power", String.format("%.2f FL Power", wheelPowers[0]));
        telemetry.addData("FR Power", String.format("%.2f FR Power", wheelPowers[1]));
        telemetry.addData("BL Power", String.format("%.2f BL Power", wheelPowers[2]));
        telemetry.addData("BR Power", String.format("%.2f BR Power", wheelPowers[3]));

        telemetry.addData("right_stick_x", String.format("%.2f right_stick_x", gamepad1.right_stick_x));
        telemetry.addData("right_stick_y", String.format("%.2f right_stick_y", gamepad1.right_stick_y));

        telemetry.addData("left_stick_x", String.format("%.2f left_stick_x", gamepad1.left_stick_x));
        telemetry.addData("left_stick_y", String.format("%.2f left_stick_y", gamepad1.left_stick_y));*/

        /*if (io.touchArmExtender.getState() == false) {
            telemetry.addData("Touch Arm Extender", "Is Pressed");
        } else {
            telemetry.addData("Touch Arm Extender", "Is Not Pressed");
        }

        if (io.touchArmAngle.getState() == false) {
            telemetry.addData("Touch Arm Angle", "Is Pressed");
        } else {
            telemetry.addData("Touch Arm Angle", "Is Not Pressed");
        }

        telemetry.addData("IMU Heading",  "Starting at %.2f",
                io.getIMUHeading());
        telemetry.addData("IMU1 Heading",  "Starting at %.2f",
                io.getIMU1Heading());*/

/*        if (io.touchDOMExtend.getState() == false) {
            telemetry.addData("Touch DOM Extend", "Is Pressed");
        } else {
            telemetry.addData("Touch DOM Extend", "Is Not Pressed");
        }*/



/*        if (io.touchBottom.getState() == false) {
            telemetry.addData("Touch Bottom", "Is Pressed");
        } else {
            telemetry.addData("Touch Bottom", "Is Not Pressed");
        }

        if (io.touchProximity.getState() == false) {
            telemetry.addData("Touch Proximity", "Is Pressed");
        } else {
            telemetry.addData("Touch Proximity", "Is Not Pressed");
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
        }*/
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
        io.setDrivePower(0 , 0, 0, 0);

        io.armExtenderMotor.setPower(0);
        io.armAngleMotor.setPower(0);
        /*io.dom2Motor.setPower(0);
        io.domExtendMotor.setPower(0);
        io.domSweepMotor.setPower(0);
        io.hookStop();*/
        /*io.forkLiftMotor.setPower(0);
        io.rpu1Motor.setPower(0);
        io.proximityArmUp();*/
    }

    public void addInitArmAngleCommands() {
        commandsInitArmAngle.add(new ArmAngleMovement(io.getArmAngleEncoder(), ArmAngleMovement.INCREASINGDIRECTION, .75));
    }

    /*public void addInitDOM2Commands() {
        commandsInitDOM2.add(new DOM2Movement(io.getDOMPotDegrees(), DOM2Movement.INCREASINGDIRECTION, .6));
    }*/

    /*public void addRetrieveRelicRPU1Commands() {
        commandsRetrieveRelicRPU1.add(new RPU1Movement(257, RPU1Movement.INCREASINGDIRECTION, .4));
    }

    public void addRetrieveRelicRPU2Commands() {
        commandsRetrieveRelicRPU2.add(new RPU2Movement(280, RPU2Movement.INCREASINGDIRECTION, .3));
    }

    public void addRetrieveRelicStowRPU1Commands() {
        commandsRetrieveRelicStowRPU1.add(new RPU1Movement(0, RPU1Movement.INCREASINGDIRECTION, .4));
    }

    public void addRetrieveRelicStowRPU2Commands() {
        commandsRetrieveRelicStowRPU2.add(new RPU2Movement(575, RPU2Movement.INCREASINGDIRECTION, .5));
    }

    public void addRetrieveRelicScoreRPU1Commands() {
        commandsRetrieveRelicScoreRPU1.add(new RPU1Movement(0, RPU1Movement.INCREASINGDIRECTION, .25));
    }

    public void addRetrieveRelicScoreRPU2Commands() {
        commandsRetrieveRelicScoreRPU2.add(new RPU2Movement(280, RPU2Movement.INCREASINGDIRECTION, .25));
    }*/

 /*   public void stateMachine(int smState, BasicCommand smCurrentCommand, Iterator<BasicCommand> smIterator){
        switch (smState) {
            case INIT:
                smCurrentCommand.init();
                smState = EXECUTE;
                break;
            case EXECUTE:
                if (smCurrentCommand.isFinished()) {
                    smCurrentCommand.stop();
                    if (iteratorInit.hasNext()) {
                        smCurrentCommand = smIterator.next();
                        smState = INIT;
                    } else smState = EXECUTE; //was FINISHED
                    break;
                }
                smCurrentCommand.execute();
                break;
            case STOP:
                smCurrentCommand.stop();
                if (iteratorInit.hasNext()) {
                    smCurrentCommand = smIterator.next();
                    smState = INIT;
                } else smState = FINISHED;
                break;
            case FINISHED:
                break;
        }
    }*/

}
