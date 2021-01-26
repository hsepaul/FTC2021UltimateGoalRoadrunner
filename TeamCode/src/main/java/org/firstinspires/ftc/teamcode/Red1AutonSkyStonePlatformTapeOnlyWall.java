package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.CalibrationSkystoneCommand;
import org.firstinspires.ftc.teamcode.commands.CommandGroup;
import org.firstinspires.ftc.teamcode.commands.DriveForwardHeavySkyStonewithSlideLeft;
import org.firstinspires.ftc.teamcode.commands.DriveForwardSkyStone;
import org.firstinspires.ftc.teamcode.commands.DriveSidewaysSkyStoneMecanumNoSkystones;
import org.firstinspires.ftc.teamcode.commands.GripperRotateSlightlyUp;
import org.firstinspires.ftc.teamcode.commands.HooksDown;
import org.firstinspires.ftc.teamcode.commands.HooksUp;
import org.firstinspires.ftc.teamcode.commands.ResetDriveEncoders;
import org.firstinspires.ftc.teamcode.commands.RotateHeavy;
import org.firstinspires.ftc.teamcode.commands.WaitForTime;

/**
 * Created by David Austin on 11/10/2016.
 */

@Autonomous(name="Red Tape Only Wall",group="Red Auton")
public class Red1AutonSkyStonePlatformTapeOnlyWall extends RedAutonSkyStone {
    public void addFinalCommands() {
        commands.add(new WaitForTime(5000));


        commands.add(new GripperRotateSlightlyUp(3000));
        commands.add(new WaitForTime(500));
        commands.add(new ResetDriveEncoders());

        commands.add(new DriveForwardSkyStone(-24,DriveForwardSkyStone.XLESSTHAN,-.5,0, 4000));
        commands.add(new WaitForTime(500));
        commands.add(new ResetDriveEncoders());

        commands.add( new CalibrationSkystoneCommand(8000));

        commands.add(new WaitForTime(250));
        commands.add(new ResetDriveEncoders());
        //commands.add(new DriveForward(18,DriveForward.XGREATERTHAN,.8,0, false, true, true));
    }
}
