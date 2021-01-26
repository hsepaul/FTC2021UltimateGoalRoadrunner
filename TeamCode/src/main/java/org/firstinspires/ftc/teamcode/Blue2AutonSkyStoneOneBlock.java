package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.ArmAngleDown;
import org.firstinspires.ftc.teamcode.commands.ArmAngleUp;
import org.firstinspires.ftc.teamcode.commands.CalibrationSkystoneCommand;
import org.firstinspires.ftc.teamcode.commands.CommandGroup;
import org.firstinspires.ftc.teamcode.commands.DriveForwardHeadingandDistanceSensorSkyStone;
import org.firstinspires.ftc.teamcode.commands.DriveForwardSkyStone;
import org.firstinspires.ftc.teamcode.commands.DriveSidewaysSkyStoneMecanum;
import org.firstinspires.ftc.teamcode.commands.DriveSidewaysSkyStoneMecanumNoSkystones;
import org.firstinspires.ftc.teamcode.commands.DriveSidewaysSkyStoneMecanumTensorFlow;
import org.firstinspires.ftc.teamcode.commands.GripperPincherClosed;
import org.firstinspires.ftc.teamcode.commands.GripperPincherOpen;
import org.firstinspires.ftc.teamcode.commands.GripperRotateParallel;
import org.firstinspires.ftc.teamcode.commands.ResetDriveEncoders;
import org.firstinspires.ftc.teamcode.commands.Rotate;
import org.firstinspires.ftc.teamcode.commands.WaitForTime;

/**
 * Created by David Austin on 11/10/2016.
 */

//@Autonomous(name="Blue Sky Stone Blocks (1 Block)",group="Auton")
public class Blue2AutonSkyStoneOneBlock extends BlueAutonSkyStone {
    public void addFinalCommands() {
        CommandGroup group = new CommandGroup();
        group.addCommand(new GripperRotateParallel(3000));
        group.addCommand(new GripperPincherOpen(3000));
        group.addCommand(new ArmAngleUp(5000));
        group.addCommand(new DriveForwardSkyStone(16,DriveForwardSkyStone.XGREATERTHAN,.5,0, 5000));
        commands.add(group);

        commands.add(new WaitForTime(250));
        commands.add(new ResetDriveEncoders());

        //commands.add(new GripperRotateParallel());
        //commands.add(new WaitForTime(500));
        //commands.add(new GripperPincherOpen());
        //commands.add(new WaitForTime(250));
        //commands.add((new ArmAngleUp()));
        //commands.add(new WaitForTime(500));
        //commands.add(new DriveForwardSkyStone(16,DriveForwardSkyStone.XGREATERTHAN,.5,0));
        //commands.add(new WaitForTime(250));
        //commands.add(new ResetDriveEncoders());

        commands.add(new DriveSidewaysSkyStoneMecanumTensorFlow(0,.4,0, 12000));
        commands.add(new WaitForTime(250));
        commands.add(new ResetDriveEncoders());
        commands.add(new DriveForwardHeadingandDistanceSensorSkyStone(3.5, DriveForwardHeadingandDistanceSensorSkyStone.RIGHTLESSTHAN,.4,0, 5000));
        commands.add(new WaitForTime(500));
        commands.add((new ArmAngleDown(5000)));
        commands.add(new WaitForTime(500));
        commands.add(new GripperPincherClosed(3000));
        commands.add(new WaitForTime(500));
        commands.add((new ArmAngleUp(5000)));
        //commands.add(new GripperRotateStowed());
        commands.add(new WaitForTime(250));
        commands.add(new ResetDriveEncoders());
        commands.add(new DriveForwardSkyStone(-12,DriveForwardSkyStone.XLESSTHAN,-.5,0, 5000));
        commands.add(new WaitForTime(250));
        commands.add(new ResetDriveEncoders());
        commands.add(new WaitForTime(250));
        commands.add(new DriveSidewaysSkyStoneMecanum(-1, DriveSidewaysSkyStoneMecanum.XLESSTHAN,-.85,0, 12000));
        commands.add(new WaitForTime(250));
        commands.add(new ResetDriveEncoders());
        commands.add(new GripperPincherOpen(3000));
        commands.add(new WaitForTime(250));
        //commands.add(new GripperRotateParallel());
        //commands.add((new ArmAngleDown()));
        //commands.add(new WaitForTime(500));
        commands.add(new Rotate(-90, .75, .75, 4000));
        commands.add(new WaitForTime(250));
        commands.add(new ResetDriveEncoders());
        commands.add(new DriveForwardSkyStone(-10,DriveForwardSkyStone.XLESSTHAN,-.75,-90, 5000));
        commands.add(new WaitForTime(250));
        commands.add(new ResetDriveEncoders());
        commands.add(new DriveSidewaysSkyStoneMecanumNoSkystones(8, DriveSidewaysSkyStoneMecanum.XGREATERTHAN,.5,-90, 30000));
        commands.add(new WaitForTime(250));
        commands.add(new ResetDriveEncoders());
        commands.add( new CalibrationSkystoneCommand(8000));

    }
}
