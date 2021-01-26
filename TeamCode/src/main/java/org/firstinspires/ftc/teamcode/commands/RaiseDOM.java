package org.firstinspires.ftc.teamcode.commands;

import java.util.ArrayList;
import java.util.Iterator;


/**
 * Created by HPaul on 10/22/2017.
 */

public class RaiseDOM extends BasicCommand {
    long timeOut;
    ArrayList<BasicCommand> commandsInitDOM1;
    ArrayList<BasicCommand> commandsInitDOM2;
    BasicCommand currentCommandInitDOM1;
    BasicCommand currentCommandInitDOM2;
    Iterator<BasicCommand> iteratorInitDOM1;
    Iterator<BasicCommand> iteratorInitDOM2;

    public RaiseDOM(){

    }

    public void init() {
        timeOut = System.currentTimeMillis() + 10000;
        commandsInitDOM1 = new ArrayList<BasicCommand>();
        commandsInitDOM2 = new ArrayList<BasicCommand>();

        commandsInitDOM1.add(new ArmAngleMovement(70, ArmAngleMovement.INCREASINGDIRECTION, .75));
        //commandsInitDOM2.add(new DOM2Movement(70, DOM2Movement.INCREASINGDIRECTION, .75));

        iteratorInitDOM1 = commandsInitDOM1.iterator();
        iteratorInitDOM2 = commandsInitDOM2.iterator();

        currentCommandInitDOM1 = iteratorInitDOM1.next();
        currentCommandInitDOM2 = iteratorInitDOM2.next();
    }

    public void execute(){
        telemetry.addData("Mode:", "Raise DOM");
        telemetry.addData("Heading: ", Math.toDegrees(io.heading));
        telemetry.addData("Captured Gold Heading: ", io.headingOfGold);
        currentCommandInitDOM1.execute();
        currentCommandInitDOM2.execute();
    }

    public boolean isFinished(){
        telemetry.addData("Heading: ", Math.toDegrees(io.heading));
        telemetry.addData("Captured Gold Heading: ", io.headingOfGold);
        return (io.twoCyclesIsGoldCentered && io.twoCyclesIsGoldAligned && io.twoCyclesIsGoldFound) || io.GoldMineralPositionCameraAverageComplete || System.currentTimeMillis() >= timeOut;
    }
    public void stop() {
        //io.dom1Motor.setPower(0);
        //io.dom2Motor.setPower(0);
        //io.setDrivePower(0,0);
        //io.forkLiftMotor.setPower(0);
    }

}

