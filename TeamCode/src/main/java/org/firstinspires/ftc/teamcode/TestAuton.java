package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.commands.CommandGroup;
import org.firstinspires.ftc.teamcode.commands.DriveForward;
import org.firstinspires.ftc.teamcode.commands.DriveForwardGlyph;
import org.firstinspires.ftc.teamcode.commands.ElevatorDown;
import org.firstinspires.ftc.teamcode.commands.ElevatorUp;
import org.firstinspires.ftc.teamcode.commands.HandsClosed;
import org.firstinspires.ftc.teamcode.commands.HandsOpened;
import org.firstinspires.ftc.teamcode.commands.IdentifyJewel;
import org.firstinspires.ftc.teamcode.commands.IdentifyJewelwithREVColorSensor;
import org.firstinspires.ftc.teamcode.commands.IdentifyProximity;
import org.firstinspires.ftc.teamcode.commands.IdentifyVuMark;
import org.firstinspires.ftc.teamcode.commands.JewelArmDown;
import org.firstinspires.ftc.teamcode.commands.JewelArmUp;
import org.firstinspires.ftc.teamcode.commands.ProximityArmDown;
import org.firstinspires.ftc.teamcode.commands.ProximityArmMid;
import org.firstinspires.ftc.teamcode.commands.ProximityArmUp;
import org.firstinspires.ftc.teamcode.commands.RemoveOpponentsJewel;
import org.firstinspires.ftc.teamcode.commands.ResetDriveEncoders;
import org.firstinspires.ftc.teamcode.commands.Rotate;
import org.firstinspires.ftc.teamcode.commands.WaitForTime;
import org.firstinspires.ftc.teamcode.utilities.IO_4WD_Test;

//import org.firstinspires.ftc.teamcode.commands.DriveForwardForDistance;
//import org.firstinspires.ftc.teamcode.commands.DriveHorizontal;
//import org.firstinspires.ftc.teamcode.commands.PressButtons;
//import org.firstinspires.ftc.teamcode.commands.ReadCamera;
//import org.firstinspires.ftc.teamcode.commands.ShootParticles;
//import org.firstinspires.ftc.teamcode.commands.StopAtLine;

/**
 * Created by David Austin on 11/10/2016.
 */
@Autonomous(name="Test",group="Auton")
@Disabled
public class TestAuton extends FirstAuton {
    public TestAuton() {
        super();
        //allianceColor = IO.BLUE;
    }

    @Override
    public void addCommands() {
        io.setAllianceColor(IO_4WD_Test.BLUE);
        commands.add(new ResetDriveEncoders());
        commands.add(new HandsClosed());
        commands.add(new WaitForTime(150));
        commands.add(new ElevatorUp());
        commands.add(new WaitForTime(150));
        commands.add(new ProximityArmMid());
        commands.add(new WaitForTime(150));
        commands.add(new JewelArmDown());
        commands.add(new WaitForTime(150));
        //CommandGroup group = new CommandGroup();
        //group.addCommand(new DriveHorizontal(13,-2,0.5));
        //group.addCommand(new ShootParticles());
        //commands.add(group);
        //commands.add(new WaitForTime(100));
        //commands.add(new DriveForward(47,DriveForward.YGREATERTHAN,.55,38));
        //group.addCommand(new IdentifyVuMark());
        //group.addCommand(new IdentifyJewel());
        commands.add(new IdentifyJewelwithREVColorSensor());



/*        commands.add(new ResetDriveEncoders());
        commands.add(new HandsClosed());
        commands.add(new WaitForTime(150));
        commands.add(new ElevatorUp());
        commands.add(new WaitForTime(150));
        commands.add(new ProximityArmMid());
        commands.add(new WaitForTime(150));
        commands.add(new ResetDriveEncoders());
        commands.add(new DriveForwardGlyph(.35, "Blue1"));
        commands.add(new WaitForTime(100));
        commands.add(new ResetDriveEncoders());
        commands.add(new Rotate(-90,.6,.6));
        commands.add(new WaitForTime(100));
        commands.add(new ResetDriveEncoders());
        commands.add(new ProximityArmDown());
        commands.add(new WaitForTime(50));
        commands.add(new DriveForward(-25,DriveForward.YLESSTHAN,.55,-90)); //This plus drive to glyph box w/ correction is -9
        commands.add(new ResetDriveEncoders());
        commands.add(new DriveForward(10,DriveForward.YGREATERTHAN,-.6,-90, false, false, true));
        commands.add(new WaitForTime(50));
        commands.add(new ResetDriveEncoders());
        commands.add(new IdentifyProximity());
        commands.add(new WaitForTime(50));
        commands.add(new ProximityArmUp());
        commands.add(new WaitForTime(50));
        //drive forward remaining with correction
        commands.add(new DriveForward(-4,DriveForward.YLESSTHAN,.55,-90, false, true)); //This plus drive to glyph box w/ correction is -9
        commands.add(new WaitForTime(50));
        commands.add(new ResetDriveEncoders());
        //commands.add(new ElevatorDown());
        //commands.add(new WaitForTime(250));
        //commands.add(new ResetDriveEncoders());
        commands.add(new HandsOpened());
        commands.add(new WaitForTime(50));
        commands.add(new ResetDriveEncoders());
        commands.add(new DriveForward(2,DriveForward.YGREATERTHAN,-.6,-90));
        commands.add(new WaitForTime(50));
        commands.add(new ResetDriveEncoders());*/



/*        commands.add(new ProximityArmMid());
        commands.add(new WaitForTime(150));
        commands.add(new JewelArmDown());
        commands.add(new WaitForTime(150));
        commands.add(new IdentifyJewelwithREVColorSensor());*/

     /*   commands.add(new ResetDriveEncoders());
        commands.add(new HandsClosed());
        commands.add(new WaitForTime(250));
        commands.add(new ElevatorUp());
        commands.add(new WaitForTime(500));
        commands.add(new ProximityArmMid());
        commands.add(new WaitForTime(500));
        commands.add(new ProximityArmDown());
        commands.add(new WaitForTime(1000));
        commands.add(new JewelArmDown());
        commands.add(new WaitForTime(1000));
        commands.add(new ResetDriveEncoders());
        //commands.add(new Rotate(-90,.6,.6));
        commands.add(new IdentifyProximity());
        commands.add(new WaitForTime(1000));
        commands.add(new ResetDriveEncoders());
        commands.add(new JewelArmUp());
        commands.add(new WaitForTime(2000));
        commands.add(new ProximityArmUp());
        commands.add(new WaitForTime(1000));
        commands.add(new DriveForward(-9,DriveForward.YLESSTHAN,.5,-90, false, true));
        //commands.add(new ElevatorDown());
        //commands.add(new WaitForTime(250));
        //commands.add(new ResetDriveEncoders());
        commands.add(new HandsOpened());
        commands.add(new WaitForTime(500));
        commands.add(new ResetDriveEncoders());
        commands.add(new DriveForward(2,DriveForward.YGREATERTHAN,-.5,-90, false, true));
        commands.add(new WaitForTime(500));
        commands.add(new ResetDriveEncoders());*/
    }

    public void addFinalCommands() {

    }
}
