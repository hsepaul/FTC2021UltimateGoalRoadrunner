package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.ArmAngleDown;
import org.firstinspires.ftc.teamcode.commands.ArmAngleDown1000;
import org.firstinspires.ftc.teamcode.commands.ArmAngleUp;
import org.firstinspires.ftc.teamcode.commands.ArmAngleUp3000;
import org.firstinspires.ftc.teamcode.commands.CalibrationSkystoneCommand;
import org.firstinspires.ftc.teamcode.commands.CommandGroup;
import org.firstinspires.ftc.teamcode.commands.DriveForwardHeadingandDistanceSensorSkyStone;
import org.firstinspires.ftc.teamcode.commands.DriveForwardHeavySkyStonewithSlideLeft;
import org.firstinspires.ftc.teamcode.commands.DriveForwardHeavySkyStonewithSlideRight;
import org.firstinspires.ftc.teamcode.commands.DriveForwardSkyStone;
import org.firstinspires.ftc.teamcode.commands.DriveForwardSkyStoneOpenCV;
import org.firstinspires.ftc.teamcode.commands.DriveSidewaysSkyStoneMecanumNoSkystones;
import org.firstinspires.ftc.teamcode.commands.DriveSidewaysSkyStoneOpenCV;
import org.firstinspires.ftc.teamcode.commands.GripperPincherClosed;
import org.firstinspires.ftc.teamcode.commands.GripperPincherOpenArmUp;
import org.firstinspires.ftc.teamcode.commands.GripperRotateParallel;
import org.firstinspires.ftc.teamcode.commands.HooksDown;
import org.firstinspires.ftc.teamcode.commands.HooksUp;
import org.firstinspires.ftc.teamcode.commands.ResetDriveEncoders;
import org.firstinspires.ftc.teamcode.commands.Rotate;
import org.firstinspires.ftc.teamcode.commands.RotateHeavy;
import org.firstinspires.ftc.teamcode.commands.WaitForTime;

/**
 * Created by David Austin on 11/10/2016.
 */

@Autonomous(name="Blue Skystone Foundation Skybridge",group="Blue Auton")
public class Blue2AutonSkyStoneOpenCVFoundation extends BlueAutonSkyStone {
    public void addFinalCommands() {

        CommandGroup group = new CommandGroup();
        group.addCommand(new DriveForwardSkyStone(5,DriveForwardSkyStone.XGREATERTHAN,.7,0, 2000));
        group.addCommand(new GripperRotateParallel(3000));
        //group.addCommand(new GripperRotateSlightlyUp(3000));
        group.addCommand(new ArmAngleUp(5000));
        commands.add(group);

        commands.add(new WaitForTime(250));
        commands.add(new ResetDriveEncoders());

        commands.add(new DriveSidewaysSkyStoneOpenCV(io.skystoneWidth,DriveForwardSkyStone.XGREATERTHAN,.65,0, 2000));

        commands.add(new WaitForTime(250));
        commands.add(new ResetDriveEncoders());



        commands.add(new DriveForwardHeadingandDistanceSensorSkyStone(io.approachDistance+3, DriveForwardHeadingandDistanceSensorSkyStone.RIGHTLESSTHAN,.4,0, 3000));

        commands.add(new WaitForTime(250));
        commands.add(new ResetDriveEncoders());

        //GRAB SKYSTONE
        CommandGroup group1 = new CommandGroup();
        group1.addCommand(new GripperPincherClosed( 1500));
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
        group3.addCommand(new Rotate(-90, .5, .5, 3000));
        group3.addCommand(new ArmAngleDown1000(5000));
        commands.add(group3);



        //commands.add(new Rotate(90, .5, .5, 3000));

        commands.add(new WaitForTime(250));
        commands.add(new ResetDriveEncoders());

        //commands.add(new DriveSidewaysSkyStoneMecanumNoSkystones(-5,DriveSidewaysSkyStoneMecanumNoSkystones.XLESSTHAN,-.65,90, 3000));

        //commands.add(new WaitForTime(250));
        //commands.add(new ResetDriveEncoders());






        commands.add(new DriveForwardSkyStoneOpenCV((io.distanceToTape+io.distancePastTape+22),DriveForwardSkyStone.XGREATERTHAN,.75,-90, 10000));

        commands.add(new WaitForTime(250));
        commands.add(new ResetDriveEncoders());

        commands.add(new Rotate(0, .5, .5, 3000));

        commands.add(new WaitForTime(250));
        commands.add(new ResetDriveEncoders());

        commands.add(new DriveForwardSkyStone(9,DriveForwardSkyStone.XGREATERTHAN,.65,0, 8000));
        commands.add(new WaitForTime(250));
        commands.add(new ResetDriveEncoders());

        commands.add(new HooksDown(3000));
        commands.add(new WaitForTime(1000));

        //DROP SKYSTONE
        commands.add(new GripperPincherOpenArmUp( 1000));



        commands.add(new DriveForwardHeavySkyStonewithSlideRight(-14, DriveForwardHeavySkyStonewithSlideRight.XLESSTHAN,-.65,0, 8000));
        commands.add(new WaitForTime(500));
        commands.add(new ResetDriveEncoders());

        //commands.add(new RotateSkyStonePlatformRed(90, .75, .75));
        commands.add(new RotateHeavy(-90, .75, .75, 3000));
        commands.add(new WaitForTime(500));
        commands.add(new ResetDriveEncoders());

        //commands.add(new DriveSidewaysSkyStoneMecanum(30,DriveSidewaysSkyStoneMecanum.XGREATERTHAN,.85,90));
        //commands.add(new ResetDriveEncoders());
        //commands.add(new WaitForTime(500));

        //commands.add(new DriveForwardSkyStone(10,DriveForwardSkyStone.XGREATERTHAN,.5,-90, 2000));
        //commands.add(new WaitForTime(500));
        //commands.add(new ResetDriveEncoders());

        //commands.add(new HooksUp(3000));
        //commands.add(new WaitForTime(1000));


        CommandGroup group4 = new CommandGroup();
        group4.addCommand(new DriveForwardSkyStone(10,DriveForwardSkyStone.XGREATERTHAN,.5,-90, 2000));
        group4.addCommand(new HooksUp(3000));
        commands.add(group4);

        commands.add(new WaitForTime(250));
        commands.add(new ResetDriveEncoders());




        commands.add(new DriveForwardSkyStone(-3,DriveForwardSkyStone.XLESSTHAN,-.75,-90, 5000));
        commands.add(new WaitForTime(250));
        commands.add(new ResetDriveEncoders());



        commands.add(new DriveSidewaysSkyStoneMecanumNoSkystones(3,DriveSidewaysSkyStoneMecanumNoSkystones.XGREATERTHAN,.75,-90, 3000));


        commands.add(new WaitForTime(250));
        commands.add(new ResetDriveEncoders());


        CommandGroup group5 = new CommandGroup();
        group5.addCommand(new ArmAngleDown(5000));
        group5.addCommand(new DriveForwardSkyStone(-25,DriveForwardSkyStone.XLESSTHAN,-.5,-90, 3000));
        commands.add(group5);


        commands.add(new WaitForTime(250));
        commands.add(new ResetDriveEncoders());

        //commands.add(new DriveSidewaysSkyStoneMecanum(-50,DriveSidewaysSkyStoneMecanum.XLESSTHAN,-.85,0));
        commands.add( new CalibrationSkystoneCommand(8000));

        commands.add(new WaitForTime(250));
        commands.add(new ResetDriveEncoders());
        //commands.add(new DriveForward(18,DriveForward.XGREATERTHAN,.8,0, false, true, true));


    }
}
