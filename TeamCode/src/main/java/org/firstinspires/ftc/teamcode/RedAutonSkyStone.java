package org.firstinspires.ftc.teamcode;


//import org.firstinspires.ftc.teamcode.commands.AlignwithGoldMineral;

import org.firstinspires.ftc.teamcode.commands.ResetDriveEncoders;
import org.firstinspires.ftc.teamcode.commands.SetIMUOffset;
import org.firstinspires.ftc.teamcode.utilities.IO_SkyStone_Test;

//import org.firstinspires.ftc.teamcode.commands.FindGoldMineral;
//import org.firstinspires.ftc.teamcode.commands.IdentifyGoldMineral;

//import org.firstinspires.ftc.teamcode.commands.DriveForwardForDistance;
//import org.firstinspires.ftc.teamcode.commands.DriveHorizontal;
//import org.firstinspires.ftc.teamcode.commands.PressButtons;
//import org.firstinspires.ftc.teamcode.commands.ReadCamera;
//import org.firstinspires.ftc.teamcode.commands.ShootParticles;
//import org.firstinspires.ftc.teamcode.commands.StopAtLine;

/**
 * Created by David Austin on 11/10/2016.
 */
public abstract class RedAutonSkyStone extends FirstAuton {
    public RedAutonSkyStone() {
        super();
        //allianceColor = IO.RED;
    }

    @Override
    public void addCommands() {
        io.setAllianceColor(IO_SkyStone_Test.RED);
        //commands.add(new RobotDown());
        //commands.add(new WaitForTime(50));
        //commands.add(new HookRelease());
        //commands.add(new ChinDown());
        //commands.add(new HookHome());
        commands.add(new ResetDriveEncoders());
        //commands.add(new ReleaseCage());
        //commands.add(new CageHome());
        commands.add(new SetIMUOffset());

        //CommandGroup group = new CommandGroup();
        //group.addCommand(new IdentifyGoldMineral());
        //group.addCommand(new FindGoldMineral());
        //group.addCommand(new AlignwithGoldMineral());
        //commands.add(group);
        //commands.add(new WaitForTime(250));
        //commands.add(new ResetDriveEncoders());
    }
}
