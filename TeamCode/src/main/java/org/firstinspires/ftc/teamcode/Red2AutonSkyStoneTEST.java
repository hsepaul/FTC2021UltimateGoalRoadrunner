package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.ArmAngleDown;
import org.firstinspires.ftc.teamcode.commands.ArmAngleUp;
import org.firstinspires.ftc.teamcode.commands.CalibrationSkystoneCommand;
import org.firstinspires.ftc.teamcode.commands.DriveForwardHeadingandDistanceSensorSkyStone;
import org.firstinspires.ftc.teamcode.commands.DriveForwardHeavySkyStonewithSlideLeft;
import org.firstinspires.ftc.teamcode.commands.DriveForwardHeavySkyStonewithSlideRight;
import org.firstinspires.ftc.teamcode.commands.DriveForwardSkyStone;
import org.firstinspires.ftc.teamcode.commands.DriveSidewaysSkyStoneMecanum;
import org.firstinspires.ftc.teamcode.commands.DriveSidewaysSkyStoneMecanumTensorFlow;
import org.firstinspires.ftc.teamcode.commands.GripperPincherClosed;
import org.firstinspires.ftc.teamcode.commands.GripperPincherOpen;
import org.firstinspires.ftc.teamcode.commands.GripperRotateParallel;
import org.firstinspires.ftc.teamcode.commands.GripperRotateStowed;
import org.firstinspires.ftc.teamcode.commands.ResetDriveEncoders;
import org.firstinspires.ftc.teamcode.commands.WaitForTime;

/**
 * Created by David Austin on 11/10/2016.
 */

//@Autonomous(name="Red Sky Stone Blocks TEST",group="Auton")
public class Red2AutonSkyStoneTEST extends RedAutonSkyStone {
    public void addFinalCommands() {
        commands.add(new DriveForwardHeavySkyStonewithSlideLeft(30, DriveForwardHeavySkyStonewithSlideLeft.XGREATERTHAN,.5,0, 10000));
        commands.add(new WaitForTime(500));
        commands.add(new ResetDriveEncoders());
        commands.add(new WaitForTime(5000));
        commands.add(new DriveForwardHeavySkyStonewithSlideLeft(-30, DriveForwardHeavySkyStonewithSlideLeft.XLESSTHAN,-.5,0, 10000));
        commands.add(new WaitForTime(500));
        commands.add(new ResetDriveEncoders());
        commands.add(new WaitForTime(5000));
        commands.add(new DriveForwardHeavySkyStonewithSlideRight(30, DriveForwardHeavySkyStonewithSlideRight.XGREATERTHAN,.5,0, 10000));
        commands.add(new WaitForTime(500));
        commands.add(new ResetDriveEncoders());
        commands.add(new WaitForTime(5000));
        commands.add(new DriveForwardHeavySkyStonewithSlideRight(-30, DriveForwardHeavySkyStonewithSlideRight.XLESSTHAN,-.5,0, 10000));
        commands.add(new WaitForTime(500));
        commands.add(new ResetDriveEncoders());
        //commands.add( new CalibrationSkystoneCommand(8000));
        //commands.add(new DriveSidewaysSkyStoneMecanum(-12, DriveSidewaysSkyStoneMecanum.XLESSTHAN,-.85,0));
        /*commands.add(new WaitForTime(250));
        commands.add(new ResetDriveEncoders());
        commands.add(new WaitForTime(250));
        commands.add(new DriveSidewaysSkyStoneMecanum(12, DriveSidewaysSkyStoneMecanum.XGREATERTHAN,.85,0));
        commands.add(new WaitForTime(250));
        commands.add(new ResetDriveEncoders());
        commands.add(new WaitForTime(250));*/

        /*commands.add(new DriveForwardSkyStone(-12,DriveForwardSkyStone.XLESSTHAN,-.5,0));
        commands.add(new WaitForTime(250));
        commands.add(new ResetDriveEncoders());
        commands.add(new WaitForTime(250));
        commands.add(new DriveForwardSkyStone(12,DriveForwardSkyStone.XGREATERTHAN,.5,0));
        commands.add(new WaitForTime(250));
        commands.add(new ResetDriveEncoders());
        commands.add(new WaitForTime(250));
*/

    }
}
