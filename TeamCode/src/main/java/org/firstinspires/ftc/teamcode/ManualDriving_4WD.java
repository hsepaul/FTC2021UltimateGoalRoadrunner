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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

//import org.firstinspires.ftc.teamcode.utilities.IO;
//import org.firstinspires.ftc.teamcode.utilities.IO_4WD_Test;
//import org.firstinspires.ftc.teamcode.utilities.PID;
import org.firstinspires.ftc.teamcode.commands.*;
import org.firstinspires.ftc.teamcode.utilities.*;

import java.util.ArrayList;
import java.util.Iterator;

//import org.firstinspires.ftc.teamcode.commands.RPU1Movement;

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

@TeleOp(name="Manual Driving 4WD OpMode", group="Iterative Opmode")
@Disabled
public class ManualDriving_4WD extends OpMode
{
    IO_4WD_Test io;
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
    static final double INCREMENT   = 0.05;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position

    // Define class members
    double  position = 0; //(MAX_POS - MIN_POS) / 2; // Start at halfway position
    double  position2 = 1; //(MAX_POS - MIN_POS) / 2; // Start at halfway position

    double initStartingPositionRPU1;
    double initStartingPositionRPU2;
    double relicRetrievalStartingPositionRPU1;
    double relicRetrievalStartingPositionRPU2;
    double relicScoreStartingPositionRPU1;
    double relicScoreStartingPositionRPU2;

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
    ArrayList<BasicCommand> commandsInitRPU1;
    ArrayList<BasicCommand> commandsInitRPU2;
    ArrayList<BasicCommand> commandsRetrieveRelicRPU1;
    ArrayList<BasicCommand> commandsRetrieveRelicRPU2;
    ArrayList<BasicCommand> commandsRetrieveRelicStowRPU1;
    ArrayList<BasicCommand> commandsRetrieveRelicStowRPU2;
    ArrayList<BasicCommand> commandsRetrieveRelicScoreRPU1;
    ArrayList<BasicCommand> commandsRetrieveRelicScoreRPU2;
    //BasicCommand currentCommand;
    BasicCommand currentCommandInitRPU1;
    BasicCommand currentCommandInitRPU2;
    BasicCommand currentCommandRetrieveRelicRPU1;
    BasicCommand currentCommandRetrieveRelicRPU2;
    BasicCommand currentCommandRetrieveRelicStowRPU1;
    BasicCommand currentCommandRetrieveRelicStowRPU2;
    BasicCommand currentCommandRetrieveRelicScoreRPU1;
    BasicCommand currentCommandRetrieveRelicScoreRPU2;
    //Iterator<BasicCommand> iterator;
    Iterator<BasicCommand> iteratorInitRPU1;
    Iterator<BasicCommand> iteratorInitRPU2;
    Iterator<BasicCommand> iteratorRetrieveRelicRPU1;
    Iterator<BasicCommand> iteratorRetrieveRelicRPU2;
    Iterator<BasicCommand> iteratorRetrieveRelicStowRPU1;
    Iterator<BasicCommand> iteratorRetrieveRelicStowRPU2;
    Iterator<BasicCommand> iteratorRetrieveRelicScoreRPU1;
    Iterator<BasicCommand> iteratorRetrieveRelicScoreRPU2;

    /*

     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        io = new IO_4WD_Test(hardwareMap, telemetry);
        io.retractHands();
        io.openRelicHand();
        io.jewelArmUp();
        io.proximityArmMid();
        io.resetDriveEncoders();

        //commented out due to Rover Ruckus (uncomment to use Relic Recovery)
        //BasicCommand.setIO(io);
        BasicCommand.setTelemetry(telemetry);

        //commands = new ArrayList<BasicCommand>();
        commandsInitRPU1 = new ArrayList<BasicCommand>();
        commandsInitRPU2 = new ArrayList<BasicCommand>();
        commandsRetrieveRelicRPU1 = new ArrayList<BasicCommand>();
        commandsRetrieveRelicRPU2 = new ArrayList<BasicCommand>();
        commandsRetrieveRelicStowRPU1 = new ArrayList<BasicCommand>();
        commandsRetrieveRelicStowRPU2 = new ArrayList<BasicCommand>();
        commandsRetrieveRelicScoreRPU1 = new ArrayList<BasicCommand>();
        commandsRetrieveRelicScoreRPU2 = new ArrayList<BasicCommand>();
        /*addInitRPU1Commands();
        addInitRPU2Commands();
        addRetrieveRelicRPU1Commands();
        addRetrieveRelicRPU2Commands();
        addRetrieveRelicStowRPU1Commands();
        addRetrieveRelicStowRPU2Commands();
        addRetrieveRelicScoreRPU1Commands();
        addRetrieveRelicScoreRPU2Commands();*/
        //addCommands();
        //addFinalCommands();
        //iterator = commands.iterator();
        iteratorInitRPU1 = commandsInitRPU1.iterator();
        iteratorInitRPU2 = commandsInitRPU2.iterator();
        iteratorRetrieveRelicRPU1 = commandsRetrieveRelicRPU1.iterator();
        iteratorRetrieveRelicRPU2 = commandsRetrieveRelicRPU2.iterator();
        iteratorRetrieveRelicStowRPU1 = commandsRetrieveRelicStowRPU1.iterator();
        iteratorRetrieveRelicStowRPU2 = commandsRetrieveRelicStowRPU2.iterator();
        iteratorRetrieveRelicScoreRPU1 = commandsRetrieveRelicScoreRPU1.iterator();
        iteratorRetrieveRelicScoreRPU2 = commandsRetrieveRelicScoreRPU2.iterator();
        //currentCommand = iterator.next();
        currentCommandInitRPU1 = iteratorInitRPU1.next();
        currentCommandInitRPU2 = iteratorInitRPU2.next();
        currentCommandRetrieveRelicRPU1 = iteratorRetrieveRelicRPU1.next();
        currentCommandRetrieveRelicRPU2 = iteratorRetrieveRelicRPU2.next();
        currentCommandRetrieveRelicStowRPU1 = iteratorRetrieveRelicStowRPU1.next();
        currentCommandRetrieveRelicStowRPU2 = iteratorRetrieveRelicStowRPU2.next();
        currentCommandRetrieveRelicScoreRPU1 = iteratorRetrieveRelicScoreRPU1.next();
        currentCommandRetrieveRelicScoreRPU2 = iteratorRetrieveRelicScoreRPU2.next();

        /*initStartingPositionRPU1 = ((RPU1Movement) currentCommandInitRPU1).targetPosition;
        initStartingPositionRPU2 = ((RPU2Movement) currentCommandInitRPU2).targetPosition;
        relicRetrievalStartingPositionRPU1 = ((RPU1Movement) currentCommandRetrieveRelicRPU1).targetPosition;
        relicRetrievalStartingPositionRPU2 = ((RPU2Movement) currentCommandRetrieveRelicRPU2).targetPosition;
        relicScoreStartingPositionRPU1 = ((RPU1Movement) currentCommandRetrieveRelicScoreRPU1).targetPosition;
        relicScoreStartingPositionRPU2 = ((RPU2Movement) currentCommandRetrieveRelicScoreRPU2).targetPosition;*/
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
            io.setDrivePower(leftPower, rightPower);
        } else if (low_speed_engaged) {
                io.setDrivePower(((.6 - (.4*gamepad1.left_trigger))*leftPower), ((.6 - (.4*gamepad1.left_trigger))*rightPower));
        } else {
            io.setDrivePower((leftPower*.6), (rightPower*.6));
        }

        if((io.getForkLiftMotorEncoder() <= -4000) && (gamepad2.left_stick_y < 0)) {
            io.forkLiftMotor.setPower(0);
        } else if(((io.getForkLiftMotorEncoder() >= 0) || (io.touchBottom.getState() == false)) && (gamepad2.left_stick_y > 0)){
            io.forkLiftMotor.setPower(0);
        } else{
            io.forkLiftMotor.setPower(gamepad2.left_stick_y);
        }

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
        }

        telemetry.addData("Elevator Hand Position", position);
        io.setHands(position);
        telemetry.addData("Relic Hand Position", position2);
        io.setRelicHand(position2);
        //io.setJewelArm(position);

        //stateMachine(initState, currentCommandInit, iteratorInit);

        if(gamepad2.right_bumper && !relic_retrieval_engaged_changed){
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
        }

        if (!relic_retrieval_engaged && !relic_retrieval_stow_engaged && !relic_retrieval_score_engaged) {

/*            telemetry.addData("RPU1 Target Position: ", initStartingPositionRPU1);
            telemetry.addData("RPU2 Target Position: ", initStartingPositionRPU2);*/

            currentCommandInitRPU1.execute();
            currentCommandInitRPU2.execute();
        }

        if (relic_retrieval_engaged && !relic_retrieval_stow_engaged && !relic_retrieval_score_engaged) {

/*            telemetry.addData("RPU1 Target Position: ", relicRetrievalStartingPositionRPU1);
            telemetry.addData("RPU2 Target Position: ", relicRetrievalStartingPositionRPU2);*/

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

            //((RPU1Movement) currentCommandRetrieveRelicRPU1).rpu1PID.setTarget(relicRetrievalStartingPositionRPU1);
            //((RPU2Movement) currentCommandRetrieveRelicRPU2).rpu2PID.setTarget(relicRetrievalStartingPositionRPU2);
        }

        if (relic_retrieval_stow_engaged && !relic_retrieval_engaged && !relic_retrieval_score_engaged) {
            currentCommandRetrieveRelicStowRPU1.execute();
            currentCommandRetrieveRelicStowRPU2.execute();
        }

        if (relic_retrieval_score_engaged && !relic_retrieval_stow_engaged && !relic_retrieval_engaged) {

/*            telemetry.addData("RPU1 Target Position: ", relicScoreStartingPositionRPU1);
            telemetry.addData("RPU2 Target Position: ", relicScoreStartingPositionRPU2);*/

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

            //((RPU1Movement) currentCommandRetrieveRelicScoreRPU1).rpu1PID.setTarget(relicScoreStartingPositionRPU1);
            //((RPU2Movement) currentCommandRetrieveRelicScoreRPU2).rpu2PID.setTarget(relicScoreStartingPositionRPU2);
        }


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

        telemetry.addData("Right Front Drive Encoder",  "Starting at %.2f",
                io.getRightFrontDriveEncoder());
        telemetry.addData("Left Back Drive Encoder",  "Starting at %.2f",
                io.getLeftBackDriveEncoder());




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
        io.setDrivePower(0 , 0);
        io.forkLiftMotor.setPower(0);
        io.rpu1Motor.setPower(0);
        io.proximityArmUp();
    }

    /*public void addInitRPU1Commands() {
        commandsInitRPU1.add(new RPU1Movement(0, RPU1Movement.INCREASINGDIRECTION, .25));
    }

    public void addInitRPU2Commands() {
        commandsInitRPU2.add(new RPU2Movement(0, RPU2Movement.INCREASINGDIRECTION, .25));
    }

    public void addRetrieveRelicRPU1Commands() {
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
