package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.robotcore.external.ClassFactory;


import org.firstinspires.ftc.teamcode.utilities.IO_4WD_Test;

import java.io.IOError;

/**
 * Created by HPaul on 10/25/2017.
 */

public class IdentifyJewel extends BasicCommand{

    long endTime;
    long foundRedJewel = 0;
    long foundBlueJewel = 0;

    public IdentifyJewel(){

    }

    public void init() {
        //io.colorSensor.enableLed(true);
        endTime = System.currentTimeMillis() + 7000;
        // send the info back to driver station using telemetry function.
        telemetry.addData("LED", true ? "On" : "Off");
        /*telemetry.addData("Clear", io.colorSensor.alpha());
        telemetry.addData("Red  ", io.colorSensor.red());
        telemetry.addData("Green", io.colorSensor.green());
        telemetry.addData("Blue ", io.colorSensor.blue());
        telemetry.addData("Unknown, Red, Blue: ", io.getJewelColor());*/
        telemetry.addData("Mode:", "Identify Jewel");
    }

    public void execute() {
        /*if ((io.colorSensor.red() >= 1) && (foundRedJewel == 0)) {
            foundRedJewel = System.currentTimeMillis();
        }
        if ((io.colorSensor.blue() >= 1) && (foundBlueJewel == 0)) {
            foundBlueJewel = System.currentTimeMillis();
        }*/
        telemetry.addData("LED", true ? "On" : "Off");
        /*telemetry.addData("Clear", io.colorSensor.alpha());
        telemetry.addData("Red  ", io.colorSensor.red());
        telemetry.addData("Green", io.colorSensor.green());
        telemetry.addData("Blue ", io.colorSensor.blue());
        telemetry.addData("Unknown, Red, Blue: ", io.getJewelColor());*/
        telemetry.addData("Mode:", "Identify Jewel");
    }

    public boolean isFinished(){
        if (System.currentTimeMillis() >= endTime) return true;
        /*if ((io.colorSensor.red() < 1) && (io.colorSensor.blue() < 1)) {
            io.setJewelColor(IO_4WD_Test.UNKNOWN);
            return false;
        } else if ((io.colorSensor.red() >= 1) && (foundRedJewel != 0) && (System.currentTimeMillis() >= (foundRedJewel + 2000))) {
            io.setJewelColor(IO_4WD_Test.RED);
            return true;
        } else if ((io.colorSensor.blue() >= 1) && (foundBlueJewel != 0) && (System.currentTimeMillis() >= (foundBlueJewel + 2000))) {
            io.setJewelColor(IO_4WD_Test.BLUE);
            return true;
        } else{
            io.setJewelColor(IO_4WD_Test.UNKNOWN);
            return false;
        }*/
        return true;
    }
    public void stop() {
        io.setDrivePower(0,0, 0, 0);
    }
}
