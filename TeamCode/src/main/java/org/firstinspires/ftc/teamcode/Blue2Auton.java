package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.DriveForward;
import org.firstinspires.ftc.teamcode.commands.DriveForwardGlyph;
import org.firstinspires.ftc.teamcode.commands.DriveForwardHeadingandDistanceSensor;
import org.firstinspires.ftc.teamcode.commands.HandsOpened;
import org.firstinspires.ftc.teamcode.commands.HandsOpenedMid;
import org.firstinspires.ftc.teamcode.commands.IdentifyProximity;
import org.firstinspires.ftc.teamcode.commands.MarkerboxDown;
import org.firstinspires.ftc.teamcode.commands.ProximityArmDown;
import org.firstinspires.ftc.teamcode.commands.ProximityArmUp;
import org.firstinspires.ftc.teamcode.commands.ResetDriveEncoders;
import org.firstinspires.ftc.teamcode.commands.Rotate;
import org.firstinspires.ftc.teamcode.commands.SetIMUOffset;
import org.firstinspires.ftc.teamcode.commands.WaitForTime;

/**
 * Created by David Austin on 11/10/2016.
 */

//@Autonomous(name="Blue Silver",group="Auton")
public class Blue2Auton extends BlueAuton {
    public void addFinalCommands() {
        commands.add(new DriveForward(10.5,DriveForward.XGREATERTHAN,.55,0, false, true));
        //commands.add(new WaitForTime(50));
        commands.add(new WaitForTime(500));
        commands.add(new ResetDriveEncoders());
        //commands.add(new ResetDriveEncoders());
        //commands.add(new WaitForTime(100));


        commands.add(new DriveForward(-14.25,DriveForward.XLESSTHAN,-.25,0, false, true));
        //commands.add(new WaitForTime(50));
        commands.add(new WaitForTime(250));
        commands.add(new ResetDriveEncoders());
        commands.add(new Rotate(-62.5, .60, .60, 4000));
        //commands.add(new WaitForTime(50));
        commands.add(new WaitForTime(250));
        commands.add(new ResetDriveEncoders());
        commands.add(new SetIMUOffset());

        //ADDED Distance Sensor May Need to Return non-distance sensor
        commands.add(new DriveForward(17,DriveForward.XGREATERTHAN,.65,0));
        //commands.add(new DriveForwardHeadingandDistanceSensor(15,DriveForwardHeadingandDistanceSensor.FRONTLESSTHAN,.75,0));
        //commands.add(new WaitForTime(50));
        commands.add(new WaitForTime(250));
        commands.add(new ResetDriveEncoders());
        //commands.add(new DriveForward(-2,DriveForward.XLESSTHAN,-.95,0));
        //commands.add(new WaitForTime(50));
        //commands.add(new ResetDriveEncoders());

        commands.add(new Rotate(-55, .55, .55, 4000));
        //commands.add(new WaitForTime(50));
        commands.add(new WaitForTime(250));
        commands.add(new ResetDriveEncoders());
        commands.add(new SetIMUOffset());

        //commands.add(new WaitForTime(50));
        commands.add(new DriveForward(12,DriveForward.XGREATERTHAN,.65,6.5));
        commands.add(new WaitForTime(250));
        commands.add(new ResetDriveEncoders());

        //commands.add(new DriveForward(12,DriveForward.XGREATERTHAN,.75,-5));
        //commands.add(new WaitForTime(250));
        //commands.add(new ResetDriveEncoders());

        //commands.add(new WaitForTime(50));
        commands.add(new Rotate(-15, .60, .60, 4000));
        commands.add(new WaitForTime(250));
        commands.add(new ResetDriveEncoders());

        commands.add(new DriveForward(8,DriveForward.XGREATERTHAN,.75,-15));
        commands.add(new WaitForTime(250));
        commands.add(new ResetDriveEncoders());
        //commands.add(new SetIMUOffset());

        //commands.add(new Rotate(-10, .75, .75));
        //commands.add(new WaitForTime(50));
        //commands.add(new WaitForTime(250));
        //commands.add(new ResetDriveEncoders());


        commands.add(new DriveForwardHeadingandDistanceSensor(5,DriveForwardHeadingandDistanceSensor.FRONTLESSTHAN,.55,-15));


        commands.add(new WaitForTime(250));
        commands.add(new ResetDriveEncoders());

        commands.add(new MarkerboxDown());

        commands.add(new DriveForward(-8,DriveForward.XLESSTHAN,-.65,-15));
        commands.add(new WaitForTime(250));
        commands.add(new ResetDriveEncoders());

        //commands.add(new WaitForTime(50));
        commands.add(new DriveForward(-18,DriveForward.XLESSTHAN,-.95,-10));
        commands.add(new WaitForTime(250));
        commands.add(new ResetDriveEncoders());
        commands.add(new DriveForwardHeadingandDistanceSensor(.1,DriveForwardHeadingandDistanceSensor.BACKLESSTHAN,-.35,-10));



        /*commands.add(new DriveForward(-12,DriveForward.XLESSTHAN,-.8,0, false, true));
        commands.add(new ResetDriveEncoders());
        commands.add(new WaitForTime(50));
        commands.add(new Rotate(-90,.65,.65));
        commands.add(new ResetDriveEncoders());*/


        /*commands.add(new ResetDriveEncoders());
        commands.add(new WaitForTime(100));
        commands.add(new DriveForward(25,DriveForward.XGREATERTHAN,.55,0));
        commands.add(new WaitForTime(100));
        commands.add(new ResetDriveEncoders());
        commands.add(new WaitForTime(100));
        commands.add(new Rotate(90,.6,.6));
        commands.add(new WaitForTime(100));
        commands.add(new ResetDriveEncoders());
        commands.add(new DriveForwardGlyph(.35, "Blue2"));
        commands.add(new WaitForTime(100));
        commands.add(new ResetDriveEncoders());

        commands.add(new WaitForTime(100));
        commands.add(new Rotate(0,.6,.6));
        commands.add(new WaitForTime(100));
        commands.add(new ResetDriveEncoders());
        commands.add(new WaitForTime(100));
        commands.add(new ProximityArmDown());
        commands.add(new WaitForTime(50));
        commands.add(new DriveForward(1,DriveForward.XGREATERTHAN,.85,0)); //This plus drive to glyph box w/ correction is 8

        commands.add(new ResetDriveEncoders());
        commands.add(new WaitForTime(100));
        //commands.add(new DriveForward(-1,DriveForward.XLESSTHAN,-.6,-90, false, false, true));
        commands.add(new WaitForTime(50));

        commands.add(new ResetDriveEncoders());
        commands.add(new WaitForTime(50));
        commands.add(new IdentifyProximity());
        commands.add(new WaitForTime(50));
        commands.add(new ProximityArmUp());
        commands.add(new WaitForTime(50));
        //drive forward remaining with correction
        commands.add(new DriveForward(3.5,DriveForward.XGREATERTHAN,.75,0, false, true)); //This plus drive to glyph box w/ correction is 8
        commands.add(new WaitForTime(50));
        commands.add(new ResetDriveEncoders());
        commands.add(new WaitForTime(100));
        //commands.add(new ElevatorDown());
        //commands.add(new WaitForTime(250));
        //commands.add(new ResetDriveEncoders());
        commands.add(new HandsOpenedMid());
        commands.add(new WaitForTime(50));
        commands.add(new ResetDriveEncoders());
        commands.add(new WaitForTime(100));
        commands.add(new DriveForward(-2,DriveForward.XLESSTHAN,-.6,0));
        commands.add(new WaitForTime(50));
        commands.add(new ResetDriveEncoders());
        commands.add(new WaitForTime(100));
*//*        double angle = -70;
        commands.add(new Rotate(angle,0.9,0));
        commands.add(new DriveForward(25, DriveForward.YLESSTHAN, 0.7, angle));
        double finalAngle = -135;
        commands.add(new Rotate(finalAngle,0.9,0));
        commands.add(new DriveForward(0, DriveForward.YLESSTHAN, 0.7, finalAngle));*/
    }
}
