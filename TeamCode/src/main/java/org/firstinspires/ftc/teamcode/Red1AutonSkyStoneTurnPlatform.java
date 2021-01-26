package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.ArmAngleUp;
import org.firstinspires.ftc.teamcode.commands.CalibrationSkystoneCommand;
import org.firstinspires.ftc.teamcode.commands.CommandGroup;
import org.firstinspires.ftc.teamcode.commands.DriveForwardHeavySkyStonewithSlideLeft;
import org.firstinspires.ftc.teamcode.commands.DriveForwardHeavySkyStonewithSlideRight;
import org.firstinspires.ftc.teamcode.commands.DriveForwardSkyStone;
import org.firstinspires.ftc.teamcode.commands.DriveSidewaysSkyStoneMecanum;
import org.firstinspires.ftc.teamcode.commands.DriveSidewaysSkyStoneMecanumNoSkystones;
import org.firstinspires.ftc.teamcode.commands.GripperRotateParallel;
import org.firstinspires.ftc.teamcode.commands.GripperRotateSlightlyUp;
import org.firstinspires.ftc.teamcode.commands.HooksDown;
import org.firstinspires.ftc.teamcode.commands.HooksUp;
import org.firstinspires.ftc.teamcode.commands.ResetDriveEncoders;
import org.firstinspires.ftc.teamcode.commands.Rotate;
import org.firstinspires.ftc.teamcode.commands.RotateHeavy;
import org.firstinspires.ftc.teamcode.commands.RotateSkyStonePlatformRed;
import org.firstinspires.ftc.teamcode.commands.WaitForTime;

/**
 * Created by David Austin on 11/10/2016.
 */

@Autonomous(name="Red Foundation Skybridge",group="Red Auton")
public class Red1AutonSkyStoneTurnPlatform extends RedAutonSkyStone {
    public void addFinalCommands() {
        commands.add(new DriveForwardSkyStone(2,DriveForwardSkyStone.XGREATERTHAN,.5,0, 1000));
        commands.add(new WaitForTime(250));
        commands.add(new ResetDriveEncoders());

        commands.add(new DriveSidewaysSkyStoneMecanumNoSkystones(11,DriveSidewaysSkyStoneMecanumNoSkystones.XGREATERTHAN,.85,0, 5000));
        commands.add(new WaitForTime(250));
        commands.add(new ResetDriveEncoders());

        commands.add(new DriveForwardSkyStone(29,DriveForwardSkyStone.XGREATERTHAN,.5,0, 8000));
        commands.add(new WaitForTime(500));
        commands.add(new ResetDriveEncoders());

        commands.add(new HooksDown(3000));
        commands.add(new WaitForTime(1500));

        commands.add(new DriveForwardHeavySkyStonewithSlideLeft(-16, DriveForwardHeavySkyStonewithSlideLeft.XLESSTHAN,-.65,0, 8000));
        commands.add(new WaitForTime(500));
        commands.add(new ResetDriveEncoders());

        //commands.add(new RotateSkyStonePlatformRed(90, .75, .75));
        commands.add(new RotateHeavy(90, .75, .75, 3000));
        commands.add(new WaitForTime(500));
        commands.add(new ResetDriveEncoders());

        //commands.add(new DriveSidewaysSkyStoneMecanum(30,DriveSidewaysSkyStoneMecanum.XGREATERTHAN,.85,90));
        //commands.add(new ResetDriveEncoders());
        //commands.add(new WaitForTime(500));

        //commands.add(new DriveForwardSkyStone(8,DriveForwardSkyStone.XGREATERTHAN,.5,90, 2000));
        //commands.add(new WaitForTime(500));
        //commands.add(new ResetDriveEncoders());

        //commands.add(new HooksUp(3000));
        //commands.add(new WaitForTime(1500));


        CommandGroup group = new CommandGroup();
        group.addCommand(new DriveForwardSkyStone(8,DriveForwardSkyStone.XGREATERTHAN,.5,90, 2000));
        group.addCommand(new HooksUp(3000));
        commands.add(group);

        commands.add(new WaitForTime(250));
        commands.add(new ResetDriveEncoders());


        commands.add(new DriveForwardSkyStone(-3,DriveForwardSkyStone.XLESSTHAN,-.5,90, 5000));
        commands.add(new WaitForTime(500));
        commands.add(new ResetDriveEncoders());


        CommandGroup group1 = new CommandGroup();
        group1.addCommand(new GripperRotateSlightlyUp(3000));
        group1.addCommand(new DriveSidewaysSkyStoneMecanumNoSkystones(-7,DriveSidewaysSkyStoneMecanumNoSkystones.XLESSTHAN,-.65,90, 5000));
        commands.add(group1);

        commands.add(new WaitForTime(500));
        commands.add(new ResetDriveEncoders());

        commands.add(new DriveForwardSkyStone(-30,DriveForwardSkyStone.XLESSTHAN,-.5,90, 4000));
        commands.add(new WaitForTime(500));
        commands.add(new ResetDriveEncoders());

        //commands.add(new DriveSidewaysSkyStoneMecanum(-50,DriveSidewaysSkyStoneMecanum.XLESSTHAN,-.85,0));
        commands.add( new CalibrationSkystoneCommand(8000));

        commands.add(new WaitForTime(250));
        commands.add(new ResetDriveEncoders());
        //commands.add(new DriveForward(18,DriveForward.XGREATERTHAN,.8,0, false, true, true));
    }
}
