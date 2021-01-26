package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utilities.IO_SkyStone_Test;

/**
 * Created by David Austin on 10/27/2016.
 */

public class BasicCommand {
    static IO_SkyStone_Test io;
    //static IO_4WD_Test io;
    static Telemetry telemetry;
    static HardwareMap map;
    public static void setIO(IO_SkyStone_Test i) {
        io = i;
    }
    public static void setMap(HardwareMap Map) { map = Map; }
    public static IO_SkyStone_Test getIO() {
        return io ;
    }
    public static void setTelemetry(Telemetry tele) {
        telemetry = tele;
    }

    public void init(){
    }

    public void execute(){
    }

    public void stop(){
    }

    public boolean isFinished(){
        return false;
    }
}
