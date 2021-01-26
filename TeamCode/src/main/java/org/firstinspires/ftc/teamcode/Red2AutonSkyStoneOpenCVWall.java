package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.ArmAngleDown;
import org.firstinspires.ftc.teamcode.commands.ArmAngleDown1000;
import org.firstinspires.ftc.teamcode.commands.ArmAngleDownReturnToZero;
import org.firstinspires.ftc.teamcode.commands.ArmAngleUp;
import org.firstinspires.ftc.teamcode.commands.ArmAngleUp3000;
import org.firstinspires.ftc.teamcode.commands.CalibrationSkystoneCommand;
import org.firstinspires.ftc.teamcode.commands.CommandGroup;
import org.firstinspires.ftc.teamcode.commands.DriveForwardHeadingandDistanceSensorSkyStone;
import org.firstinspires.ftc.teamcode.commands.DriveForwardSkyStone;
import org.firstinspires.ftc.teamcode.commands.DriveForwardSkyStoneOpenCV;
import org.firstinspires.ftc.teamcode.commands.DriveSidewaysSkyStoneMecanumNoSkystones;
import org.firstinspires.ftc.teamcode.commands.DriveSidewaysSkyStoneOpenCV;
import org.firstinspires.ftc.teamcode.commands.GripperPincherClosed;
import org.firstinspires.ftc.teamcode.commands.GripperPincherOpenArmUp;
import org.firstinspires.ftc.teamcode.commands.GripperRotateParallel;
import org.firstinspires.ftc.teamcode.commands.ResetDriveEncoders;
import org.firstinspires.ftc.teamcode.commands.Rotate;
import org.firstinspires.ftc.teamcode.commands.WaitForTime;

/**
 * Created by David Austin on 11/10/2016.
 */

//@Autonomous(name="Red Skystone Wall",group="Red Auton")
public class Red2AutonSkyStoneOpenCVWall extends RedAutonSkyStone {
    public void addFinalCommands() {

        CommandGroup group = new CommandGroup();
        group.addCommand(new DriveForwardSkyStone(5,DriveForwardSkyStone.XGREATERTHAN,.7,0, 2000));
        group.addCommand(new GripperRotateParallel(3000));
        group.addCommand(new ArmAngleUp(5000));
        commands.add(group);

        commands.add(new WaitForTime(250));
        commands.add(new ResetDriveEncoders());

        commands.add(new DriveSidewaysSkyStoneOpenCV(io.skystoneWidth,DriveForwardSkyStone.XGREATERTHAN,.5,0, 2000));

        commands.add(new WaitForTime(250));
        commands.add(new ResetDriveEncoders());



        commands.add(new DriveForwardHeadingandDistanceSensorSkyStone(io.approachDistance+2, DriveForwardHeadingandDistanceSensorSkyStone.LEFTLESSTHAN,.4,0, 3000));

        commands.add(new WaitForTime(250));
        commands.add(new ResetDriveEncoders());

        //GRAB SKYSTONE
        CommandGroup group1 = new CommandGroup();
        group1.addCommand(new GripperPincherClosed( 1250));
        group1.addCommand(new ArmAngleDown(5000));
        group1.addCommand(new DriveForwardSkyStone(-1,DriveForwardSkyStone.XLESSTHAN,-.4,0, 1000));
        commands.add(group1);

        commands.add(new WaitForTime(250));
        commands.add(new ResetDriveEncoders());

        CommandGroup group2 = new CommandGroup();
        group2.addCommand(new ArmAngleUp3000(5000));
        group2.addCommand(new DriveForwardSkyStone(-io.backupClearance,DriveForwardSkyStone.XLESSTHAN,-.5,0, 1000));
        commands.add(group2);


        commands.add(new WaitForTime(250));
        commands.add(new ResetDriveEncoders());

        CommandGroup group3 = new CommandGroup();
        group3.addCommand(new Rotate(90, .5, .5, 3000));
        group3.addCommand(new ArmAngleDown1000(5000));
        commands.add(group3);



        //commands.add(new Rotate(90, .5, .5, 3000));

        commands.add(new WaitForTime(250));
        commands.add(new ResetDriveEncoders());

        //commands.add(new DriveSidewaysSkyStoneMecanumNoSkystones(-5,DriveSidewaysSkyStoneMecanumNoSkystones.XLESSTHAN,-.65,90, 3000));

        //commands.add(new WaitForTime(250));
        //commands.add(new ResetDriveEncoders());

        commands.add(new DriveForwardSkyStoneOpenCV((io.distanceToTape+io.distancePastTape),DriveForwardSkyStone.XGREATERTHAN,.65,90, 10000));

        commands.add(new WaitForTime(250));
        commands.add(new ResetDriveEncoders());

        //DROP SKYSTONE
        commands.add(new GripperPincherOpenArmUp( 1000));

        CommandGroup group4 = new CommandGroup();
        group4.addCommand(new ArmAngleDownReturnToZero(1000));
        group4.addCommand(new DriveForwardSkyStoneOpenCV(-(io.distancePastTape+io.distanceToTape+io.distanceBetweenSkystones),DriveForwardSkyStone.XLESSTHAN,-.65,90, 10000));
        commands.add(group4);

        commands.add(new WaitForTime(250));
        commands.add(new ResetDriveEncoders());


        //commands.add(new DriveForwardSkyStoneOpenCV(-(io.distancePastTape+io.distanceToTape+io.distanceBetweenSkystones),DriveForwardSkyStone.XLESSTHAN,-.65,90, 10000));

        //commands.add(new WaitForTime(250));
        //commands.add(new ResetDriveEncoders());

        //commands.add(new DriveSidewaysSkyStoneMecanumNoSkystones(5,DriveSidewaysSkyStoneMecanumNoSkystones.XGREATERTHAN,.65,90, 3000));

        //commands.add(new WaitForTime(250));
        //commands.add(new ResetDriveEncoders());

        commands.add(new Rotate(0, .5, .5, 10000));

        commands.add(new WaitForTime(250));
        commands.add(new ResetDriveEncoders());

        commands.add(new DriveForwardHeadingandDistanceSensorSkyStone(io.approachDistance, DriveForwardHeadingandDistanceSensorSkyStone.LEFTLESSTHAN,.4,0, 3000));

        commands.add(new WaitForTime(250));
        commands.add(new ResetDriveEncoders());

        //GRAB SKYSTONE
        CommandGroup group5 = new CommandGroup();
        group5.addCommand(new GripperPincherClosed( 1250));
        group5.addCommand(new ArmAngleDown(5000));
        group5.addCommand(new DriveForwardSkyStone(-1,DriveForwardSkyStone.XLESSTHAN,-.4,0, 1000));
        commands.add(group5);

        commands.add(new WaitForTime(250));
        commands.add(new ResetDriveEncoders());

        CommandGroup group6 = new CommandGroup();
        group6.addCommand(new ArmAngleUp3000(5000));
        group6.addCommand(new DriveForwardSkyStone(-io.backupClearance,DriveForwardSkyStone.XLESSTHAN,-.5,0, 1000));
        commands.add(group6);

        commands.add(new WaitForTime(250));
        commands.add(new ResetDriveEncoders());

        CommandGroup group7 = new CommandGroup();
        group7.addCommand(new Rotate(90, .5, .5, 3000));
        group7.addCommand(new ArmAngleDown1000(5000));
        commands.add(group7);


        //commands.add(new Rotate(90, .5, .5, 3000));

        commands.add(new WaitForTime(250));
        commands.add(new ResetDriveEncoders());

        //commands.add(new DriveSidewaysSkyStoneMecanumNoSkystones(-5,DriveSidewaysSkyStoneMecanumNoSkystones.XLESSTHAN,-.65,90, 3000));

        //commands.add(new WaitForTime(250));
        //commands.add(new ResetDriveEncoders());

        commands.add(new DriveForwardSkyStoneOpenCV((io.distanceBetweenSkystones+io.distanceToTape+io.distancePastTape-2),DriveForwardSkyStone.XGREATERTHAN,.85,90, 10000));

        commands.add(new WaitForTime(250));
        commands.add(new ResetDriveEncoders());

        //DROP SKYSTONE
        commands.add(new GripperPincherOpenArmUp( 1000));

        //commands.add(new WaitForTime(100));

        //commands.add(new DriveSidewaysSkyStoneMecanumNoSkystones(0,DriveSidewaysSkyStoneMecanumNoSkystones.XLESSTHAN,-.85,90, 3000));

        //commands.add(new WaitForTime(250));
        //commands.add(new ResetDriveEncoders());

        CommandGroup group8 = new CommandGroup();
        group8.addCommand(new ArmAngleDown(5000));
        group8.addCommand(new DriveForwardSkyStone(-(io.distancePastTape-18),DriveForwardSkyStone.XLESSTHAN,-.85,90, 5000));
        commands.add(group8);

        //commands.add(new DriveForwardSkyStone(-(io.distancePastTape/2),DriveForwardSkyStone.XLESSTHAN,-.65,90, 5000));

        commands.add(new WaitForTime(250));
        commands.add(new ResetDriveEncoders());

        commands.add( new CalibrationSkystoneCommand(8000));

        commands.add(new WaitForTime(250));
        commands.add(new ResetDriveEncoders());

    }
}
