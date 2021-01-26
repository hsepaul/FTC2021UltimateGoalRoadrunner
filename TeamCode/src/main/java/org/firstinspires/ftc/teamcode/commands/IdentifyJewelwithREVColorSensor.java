package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.utilities.IO_4WD_Test;

/**
 * Created by HPaul on 10/25/2017.
 */

public class IdentifyJewelwithREVColorSensor extends BasicCommand{

    long endTime;
    long foundRedJewel = 0;
    long foundBlueJewel = 0;

    public IdentifyJewelwithREVColorSensor(){

    }

    public void init() {
        //io.revColorSensor.enableLed(true);
        endTime = System.currentTimeMillis() + 4000;
        // send the info back to driver station using telemetry function.
        telemetry.addData("LED", true ? "On" : "Off");
        /*telemetry.addData("Clear", io.revColorSensor.alpha());
        telemetry.addData("Red  ", io.revColorSensor.red());
        telemetry.addData("Green", io.revColorSensor.green());
        telemetry.addData("Blue ", io.revColorSensor.blue());
        telemetry.addData("Unknown, Red, Blue: ", io.getJewelColor());*/
        telemetry.addData("Mode:", "Identify Jewel");
    }

    public void execute() {
        /*if (((io.revColorSensor.red() - io.revColorSensor.blue()) >= 10) && (foundRedJewel == 0)) {
            foundRedJewel = System.currentTimeMillis();
        } else if (foundBlueJewel == 0) {
            foundBlueJewel = System.currentTimeMillis();
        }*/
/*        if (((io.revColorSensor.blue() - io.revColorSensor.red()) > 0) && (foundBlueJewel == 0)) {
            foundBlueJewel = System.currentTimeMillis();
        }*/
        telemetry.addData("LED", true ? "On" : "Off");
        /*telemetry.addData("Clear", io.revColorSensor.alpha());
        telemetry.addData("Red  ", io.revColorSensor.red());
        telemetry.addData("Green", io.revColorSensor.green());
        telemetry.addData("Blue ", io.revColorSensor.blue());
        telemetry.addData("Unknown, Red, Blue: ", io.getJewelColor());*/
        telemetry.addData("Mode:", "Identify Jewel");
    }

    public boolean isFinished(){
        if (System.currentTimeMillis() >= endTime) return true;
        return true;
        /*if (Math.abs(io.revColorSensor.red() - io.revColorSensor.blue()) == 0) {
            io.setJewelColor(IO_4WD_Test.UNKNOWN);
            telemetry.addData("Unknown, Red, Blue: ", io.getJewelColor());
            telemetry.addData("Mode:", "Identify Jewel");
            return false;
        } else if (((io.revColorSensor.red() - io.revColorSensor.blue()) >= 10) && (foundRedJewel != 0) && (System.currentTimeMillis() >= (foundRedJewel + 1000))) {
            io.setJewelColor(IO_4WD_Test.RED);
            telemetry.addData("Unknown, Red, Blue: ", io.getJewelColor());
            telemetry.addData("Mode:", "Identify Jewel");
            return true;
        } else if ((foundBlueJewel != 0) && (System.currentTimeMillis() >= (foundBlueJewel + 1000))) {
            io.setJewelColor(IO_4WD_Test.BLUE);
            telemetry.addData("Unknown, Red, Blue: ", io.getJewelColor());
            telemetry.addData("Mode:", "Identify Jewel");
            return true;
        } else {
            io.setJewelColor(IO_4WD_Test.UNKNOWN);
            telemetry.addData("Unknown, Red, Blue: ", io.getJewelColor());
            telemetry.addData("Mode:", "Identify Jewel");
            return false;
        }*/
/*        } else if (((io.revColorSensor.blue() - io.revColorSensor.red()) > 0) && (foundBlueJewel != 0) && (System.currentTimeMillis() >= (foundBlueJewel + 1000))) {
            io.setJewelColor(IO_4WD_Test.BLUE);
            telemetry.addData("Unknown, Red, Blue: ", io.getJewelColor());
            telemetry.addData("Mode:", "Identify Jewel");
            return true;
        } else{
            io.setJewelColor(IO_4WD_Test.UNKNOWN);
            telemetry.addData("Unknown, Red, Blue: ", io.getJewelColor());
            telemetry.addData("Mode:", "Identify Jewel");
            return false;
        }*/
    }
    public void stop() {
        io.setDrivePower(0,0, 0, 0);
    }
}
