package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * Created by David Austin on 11/10/2016.
 */

@Autonomous(name="Follow the Gold",group="Auton")
@Disabled
public class Blue1AutonTest extends BlueAutonTest {
    public void addFinalCommands() {
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
