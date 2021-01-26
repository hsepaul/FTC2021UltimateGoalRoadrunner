package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.commands.DriveForward;
import org.firstinspires.ftc.teamcode.commands.DriveForwardGlyph;
import org.firstinspires.ftc.teamcode.commands.HandsOpened;
import org.firstinspires.ftc.teamcode.commands.ResetDriveEncoders;
import org.firstinspires.ftc.teamcode.commands.Rotate;
import org.firstinspires.ftc.teamcode.commands.WaitForTime;

/**
 * Created by David Austin on 11/10/2016.
 */

//@Autonomous(name="Blue1",group="Auton")
public class Blue1Autonold extends BlueAutonold {
    public void addFinalCommands() {
        commands.add(new ResetDriveEncoders());
        commands.add(new DriveForwardGlyph(.35, "Blue1"));
        commands.add(new WaitForTime(250));
        commands.add(new ResetDriveEncoders());
        commands.add(new Rotate(-90,.6,.6, 4000));
        commands.add(new WaitForTime(250));
        commands.add(new ResetDriveEncoders());
        commands.add(new DriveForward(-9,DriveForward.YLESSTHAN,.5,-90));
        commands.add(new WaitForTime(250));
        commands.add(new ResetDriveEncoders());
        //commands.add(new ElevatorDown());
        //commands.add(new WaitForTime(250));
        //commands.add(new ResetDriveEncoders());
        commands.add(new HandsOpened());
        commands.add(new WaitForTime(500));
        commands.add(new ResetDriveEncoders());
        commands.add(new DriveForward(2,DriveForward.YGREATERTHAN,-.5,-90));
        commands.add(new WaitForTime(500));
        commands.add(new ResetDriveEncoders());
/*        double angle = -70;
        commands.add(new Rotate(angle,0.9,0));
        commands.add(new DriveForward(25, DriveForward.YLESSTHAN, 0.7, angle));
        double finalAngle = -135;
        commands.add(new Rotate(finalAngle,0.9,0));
        commands.add(new DriveForward(0, DriveForward.YLESSTHAN, 0.7, finalAngle));*/
    }
}
