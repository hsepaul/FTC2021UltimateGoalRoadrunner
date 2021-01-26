package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.CalibrationSkystoneCommand;
import org.firstinspires.ftc.teamcode.commands.DriveForwardHeavySkyStonewithSlideLeft;
import org.firstinspires.ftc.teamcode.commands.DriveForwardSkyStone;
import org.firstinspires.ftc.teamcode.commands.DriveSidewaysSkyStoneMecanum;
import org.firstinspires.ftc.teamcode.commands.HooksDown;
import org.firstinspires.ftc.teamcode.commands.HooksUp;
import org.firstinspires.ftc.teamcode.commands.ResetDriveEncoders;
import org.firstinspires.ftc.teamcode.commands.WaitForTime;

/**
 * Created by David Austin on 11/10/2016.
 */

//@Autonomous(name="Blue Sky Stone Platform",group="Auton")
public class Blue1AutonSkyStone extends BlueAutonSkyStone {
    public void addFinalCommands() {
        commands.add(new DriveSidewaysSkyStoneMecanum(-12,DriveSidewaysSkyStoneMecanum.XLESSTHAN,-.85,0, 12000));
        commands.add(new WaitForTime(250));
        commands.add(new ResetDriveEncoders());
        commands.add(new WaitForTime(250));
        commands.add(new DriveForwardSkyStone(30,DriveForwardSkyStone.XGREATERTHAN,.5,0, 5000));
        commands.add(new ResetDriveEncoders());
        commands.add(new WaitForTime(500));
        commands.add(new HooksDown(3000));
        commands.add(new WaitForTime(1500));
        commands.add(new DriveForwardHeavySkyStonewithSlideLeft(-30, DriveForwardHeavySkyStonewithSlideLeft.XLESSTHAN,-.75,0, 5000));
        commands.add(new WaitForTime(500));
        commands.add(new HooksUp(3000));
        commands.add(new WaitForTime(1500));
        commands.add(new ResetDriveEncoders());
        commands.add(new WaitForTime(250));
        commands.add(new DriveSidewaysSkyStoneMecanum(50,DriveSidewaysSkyStoneMecanum.XGREATERTHAN,.85,0, 12000));
        commands.add( new CalibrationSkystoneCommand(8000));
        //commands.add(new DriveForward(18,DriveForward.XGREATERTHAN,.8,0, false, true, true));
    }
}
