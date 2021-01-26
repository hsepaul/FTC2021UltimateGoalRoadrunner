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

//@Autonomous(name="Red Gold",group="Auton")
public class Red1Auton extends RedAuton {
    public void addFinalCommands() {
        //commands.add(new DriveForward(18,DriveForward.XGREATERTHAN,.8,0, false, true, true));
        commands.add(new DriveForward(20,DriveForward.XGREATERTHAN,.65,0, false, true));
        //commands.add(new WaitForTime(50));

        commands.add(new WaitForTime(250));
        commands.add(new ResetDriveEncoders());
        //commands.add(new WaitForTime(50));


        commands.add(new Rotate(0, .6, .6,4000, true));
        commands.add(new WaitForTime(250));
        commands.add(new ResetDriveEncoders());
        //commands.add(new SetIMUOffset());
        commands.add(new DriveForward(13,DriveForward.XGREATERTHAN,.55,0, false, true, true));
        //commands.add(new WaitForTime(50));
        commands.add(new WaitForTime(250));
        commands.add(new ResetDriveEncoders());
        //commands.add(new MarkerboxDown());
        //MIGHT NEED TO BRING BACK REVERSE -.25
        //commands.add(new DriveForward(-1.5,DriveForward.XLESSTHAN,-.65,0, false, true, true));
        //commands.add(new WaitForTime(250));
        //commands.add(new ResetDriveEncoders());
        //commands.add(new WaitForTime(50));

        commands.add(new Rotate(-135, .55, .55, 4000));
        //commands.add(new WaitForTime(50));
        commands.add(new WaitForTime(250));
        commands.add(new ResetDriveEncoders());

        commands.add(new SetIMUOffset());

        commands.add(new DriveForward(-10,DriveForward.XLESSTHAN,-.65,0));
        commands.add(new WaitForTime(250));
        commands.add(new ResetDriveEncoders());
        //commands.add(new WaitForTime(50));
        commands.add(new MarkerboxDown());
        commands.add(new WaitForTime(100));
        //commands.add(new Rotate(-10, .75, .75));
        //commands.add(new ResetDriveEncoders());
        //commands.add(new WaitForTime(100));
        commands.add(new DriveForward(2.5,DriveForward.XGREATERTHAN,.65,-10));
        //commands.add(new WaitForTime(50));
        commands.add(new WaitForTime(250));
        commands.add(new ResetDriveEncoders());
        commands.add(new Rotate(18.5, .55, .55, 4000));
        commands.add(new WaitForTime(250));
        commands.add(new ResetDriveEncoders());
        commands.add(new DriveForward(12,DriveForward.XGREATERTHAN,.65,18.5));
        commands.add(new WaitForTime(250));
        commands.add(new ResetDriveEncoders());
        commands.add(new Rotate(10, .65, .65, 4000));
        commands.add(new WaitForTime(250));
        commands.add(new ResetDriveEncoders());

        commands.add(new DriveForward(35,DriveForward.XGREATERTHAN,.65,7.5));
        //commands.add(new WaitForTime(50));
        commands.add(new WaitForTime(250));
        commands.add(new ResetDriveEncoders());

        //commands.add(new WaitForTime(50));
        commands.add(new DriveForwardHeadingandDistanceSensor(.1,DriveForwardHeadingandDistanceSensor.FRONTLESSTHAN,.35,5));
        //commands.add(new Rotate(90,.65,.65));
        //commands.add(new ResetDriveEncoders());
        //commands.add(new DriveForward(18,DriveForward.XGREATERTHAN,.35,0));
        //commands.add(new WaitForTime(200));
        /*commands.add(new ResetDriveEncoders());
        commands.add(new WaitForTime(100));
        commands.add(new DriveForwardGlyph(.35, "Blue1"));
        commands.add(new WaitForTime(100));
        commands.add(new ResetDriveEncoders());
        commands.add(new WaitForTime(100));
        commands.add(new Rotate(-90,.6,.6));
        commands.add(new WaitForTime(100));
        commands.add(new ResetDriveEncoders());
        commands.add(new WaitForTime(100));
        commands.add(new ProximityArmDown());
        commands.add(new WaitForTime(50));
        commands.add(new DriveForward(-.8,DriveForward.YLESSTHAN,.85,-90)); //This plus drive to glyph box w/ correction is -9
        commands.add(new ResetDriveEncoders());
        commands.add(new WaitForTime(100));
        commands.add(new DriveForward(1,DriveForward.YGREATERTHAN,-.6,-90, false, false, true));
        commands.add(new WaitForTime(50));
        commands.add(new ResetDriveEncoders());
        commands.add(new WaitForTime(100));
        commands.add(new IdentifyProximity());
        commands.add(new WaitForTime(50));
        commands.add(new ProximityArmUp());
        commands.add(new WaitForTime(50));*/



        //drive forward remaining with correction
        /*commands.add(new DriveForward(-3.5,DriveForward.YLESSTHAN,.75,-90, false, true)); //This plus drive to glyph box w/ correction is -9
        commands.add(new WaitForTime(50));
        commands.add(new ResetDriveEncoders());
        commands.add(new WaitForTime(100));*/



        //commands.add(new ElevatorDown());
        //commands.add(new WaitForTime(250));
        //commands.add(new ResetDriveEncoders());



        /*commands.add(new HandsOpenedMid());
        commands.add(new WaitForTime(50));
        commands.add(new ResetDriveEncoders());
        commands.add(new WaitForTime(100));
        commands.add(new DriveForward(2,DriveForward.YGREATERTHAN,-.6,-90));
        commands.add(new WaitForTime(50));
        commands.add(new ResetDriveEncoders());
        commands.add(new WaitForTime(100));*/



/*        double angle = -70;
        commands.add(new Rotate(angle,0.9,0));
        commands.add(new DriveForward(25, DriveForward.YLESSTHAN, 0.7, angle));
        double finalAngle = -135;
        commands.add(new Rotate(finalAngle,0.9,0));
        commands.add(new DriveForward(0, DriveForward.YLESSTHAN, 0.7, finalAngle));*/
    }
}
