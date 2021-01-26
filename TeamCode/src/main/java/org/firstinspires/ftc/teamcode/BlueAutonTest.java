package org.firstinspires.ftc.teamcode;


import org.firstinspires.ftc.teamcode.commands.AlignwithGoldMineralTest;
import org.firstinspires.ftc.teamcode.commands.CommandGroup;
import org.firstinspires.ftc.teamcode.commands.DriveForward;
//import org.firstinspires.ftc.teamcode.commands.FindGoldMineral;
//import org.firstinspires.ftc.teamcode.commands.IdentifyGoldMineral;
import org.firstinspires.ftc.teamcode.commands.RaiseDOM;
import org.firstinspires.ftc.teamcode.commands.ResetDriveEncoders;
import org.firstinspires.ftc.teamcode.commands.SetIMUOffset;
import org.firstinspires.ftc.teamcode.commands.WaitForTime;
import org.firstinspires.ftc.teamcode.utilities.IO_SkyStone_Test;

//import org.firstinspires.ftc.teamcode.commands.DriveForwardForDistance;
//import org.firstinspires.ftc.teamcode.commands.DriveHorizontal;
//import org.firstinspires.ftc.teamcode.commands.PressButtons;
//import org.firstinspires.ftc.teamcode.commands.ReadCamera;
//import org.firstinspires.ftc.teamcode.commands.ShootParticles;
//import org.firstinspires.ftc.teamcode.commands.StopAtLine;

/**
 * Created by David Austin on 11/10/2016.
 */
public abstract class BlueAutonTest extends FirstAuton {
    public BlueAutonTest() {
        super();
        //allianceColor = IO.BLUE;
    }

    @Override
    public void addCommands() {
        io.setAllianceColor(IO_SkyStone_Test.BLUE);
        //commands.add(new RobotDown());
        //commands.add(new WaitForTime(50));
        //commands.add(new HookRelease());
        //commands.add(new ChinDown());
        //commands.add(new HookHome());
        commands.add(new ResetDriveEncoders());
        commands.add(new SetIMUOffset());
        //commands.add(new RaiseDOM());


        CommandGroup group = new CommandGroup();
        group.addCommand(new RaiseDOM());
        //group.addCommand(new IdentifyGoldMineral());
        //group.addCommand(new FindGoldMineral());
        group.addCommand(new AlignwithGoldMineralTest());
        //group.addCommand(new AlignwithGoldMineral());
        commands.add(group);




        //commands.add(new DriveForwardDistanceSensorandEncoder(50, DriveForwardDistanceSensorandEncoder.XGREATERTHAN,.6,10, "Right"));
        //commands.add(new DriveForwardDistanceSensorandDistanceSensor(5, DriveForwardDistanceSensorandDistanceSensor.FRONTLESSTHAN,.6,10, "Right"));
        //commands.add(new DriveForwardHeadingandDistanceSensor(5, DriveForwardHeadingandDistanceSensor.FRONTLESSTHAN,.6,20));

        commands.add(new DriveForward(15,DriveForward.XGREATERTHAN,.8,0, false, true));
        commands.add(new WaitForTime(1000));
        commands.add(new ResetDriveEncoders());
        commands.add(new DriveForward(5,DriveForward.XGREATERTHAN,.8,0, false, true, true));




        //commands.add(new ProximityArmMid());

        //CommandGroup group = new CommandGroup();
        //group.addCommand(new ElevatorUp());
        //group.addCommand(new ProximityArmMid());
        //group.addCommand(new JewelArmDown());
        //commands.add(group);
        //commands.add(new WaitForTime(150));

        //commands.add(new HandsClosed());
        //commands.add(new WaitForTime(150));
        //commands.add(new ElevatorUp());
        //commands.add(new WaitForTime(150));
        //commands.add(new ProximityArmMid());
        //commands.add(new WaitForTime(150));
        //commands.add(new JewelArmDown());
        //commands.add(new WaitForTime(150));



        //CommandGroup group = new CommandGroup();
        //group.addCommand(new DriveHorizontal(13,-2,0.5));
        //group.addCommand(new ShootParticles());
        //commands.add(group);
        //commands.add(new WaitForTime(100));
        //commands.add(new DriveForward(47,DriveForward.YGREATERTHAN,.55,38));
        //group.addCommand(new IdentifyVuMark());
        //group.addCommand(new IdentifyJewel());
        //commands.add(new IdentifyJewelwithREVColorSensor());
        //commands.add(group);
        //commands.add(new RemoveOpponentsJewel());
        //commands.add(new WaitForTime(150));
        //commands.add(new ResetDriveEncoders());
        //commands.add(new JewelArmUp());
        //commands.add(new IdentifyVuMark());
        //commands.add(new IdentifyJewel());
        //commands.add(new Rotate(-15,.7,0));
        //commands.add(new WaitForTime(150));
        //commands.add(new Rotate(0,.6,.6));
        //commands.add(new WaitForTime(150));
        //commands.add(new ResetDriveEncoders());
        //BasicCommand.getIO().resetDriveEncoders();
        //io.resetDriveEncoders();
        //commands.add(new ResetDriveEncoders());
        //telemetry.addData("x after reset: ",io.getX());
        //telemetry.addData("y after reset: ",io.getY());
        //commands.add(new DriveForward(20,DriveForward.XGREATERTHAN,.65,0));;
        //commands.add(new DriveForward(20,DriveForward.XGREATERTHAN,.35,0));
        //commands.add(new WaitForTime(1000));
        //BasicCommand.getIO().resetDriveEncoders();
        //io.resetDriveEncoders();
        //commands.add(new ResetDriveEncoders());
        //telemetry.addData("x after reset: ",io.getX());
        //telemetry.addData("y after reset: ",io.getY());
        //commands.add(new DriveForward(-20,DriveForward.XLESSTHAN,-.35,0));
        //commands.add(new WaitForTime(1000));
        //commands.add(new ResetDriveEncoders());
        //commands.add(new WaitForTime(1000));
        //commands.add(new ElevatorDown());
        //commands.add(new WaitForTime(1000));
        //commands.add(new HandsOpened());
        //commands.add(new WaitForTime(1000));
        //commands.add(new WaitForTime(1000));
        //commands.add(new Rotate(90,.65,0));
        //commands.add(new WaitForTime(1000));
        //commands.add(new DriveForward(20,DriveForward.YGREATERTHAN,.65,90));
        //commands.add(new WaitForTime(1000));
        //commands.add(new Rotate(174,.65,0));
        //commands.add(new WaitForTime(1000));
        //commands.add(new DriveForward(0,DriveForward.XLESSTHAN,.65,174));
        //commands.add(new WaitForTime(1000));
        //commands.add(new Rotate(-90,.65,0));
        //commands.add(new WaitForTime(1000));
        //commands.add(new DriveForward(10,DriveForward.YLESSTHAN,.65,-90));
        //commands.add(new WaitForTime(1000));
        //commands.add(new Rotate(0,.65,0));
        //commands.add(new Rotate(0,.6,0.6));
        //commands.add(new WaitForTime(100));
        //double ycoord = 49.5;
        //commands.add(new DriveHorizontal(90, ycoord, 0.5,true));
        //commands.add(new StopAtLine(112,ycoord,0.2,StopAtLine.FRONT, true));
        //commands.add(new DriveForwardForDistance(3.5,ycoord,0.25));

        //commands.add(new WaitForTime(500));
        //group = new CommandGroup();
        //group.addCommand(new ReadCamera());
        //group.addCommand(new DriveForwardForDistance(2, ycoord, 0.25));
        //commands.add(group);
        //commands.add(new ReadCamera());
        //commands.add(new DriveForwardForDistance(2,ycoord,0.25));
        //commands.add(new PressButtons());

        //commands.add(new WaitForTime(100));
        //commands.add(new DriveHorizontal(70,48,-.6,true)); // was 70
        //commands.add(new StopAtLine(36,48,-.2, StopAtLine.FRONT));
        //commands.add(new WaitForTime(500)); // was 500
        //commands.add(new DriveForwardForDistance(5.8,48,0.25));

        //commands.add(new WaitForTime(500));
        //group = new CommandGroup();
        //group.addCommand(new ReadCamera());
        //group.addCommand(new DriveForwardForDistance(1.75, 48, 0.2));
        //commands.add(group);
        //commands.add(new PressButtons());

        //commands.add(new WaitForTime(1500));

    }
}
