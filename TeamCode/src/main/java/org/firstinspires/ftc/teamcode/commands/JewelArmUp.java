package org.firstinspires.ftc.teamcode.commands;


/**
 * Created by HPaul on 10/22/2017.
 */

public class JewelArmUp extends BasicCommand {

    boolean jewelArmUp = false;
    long timeOut;
    int loops = 0;
    static final double INCREMENT   = 0.05; //was 0.02    // amount to slew servo each CYCLE_MS cycle
    //static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_POS     =  1.00;     // Maximum rotational position
    static final double MIN_POS     =  0.00;     // Minimum rotational position
    double  position = MAX_POS;


        public JewelArmUp(){

        }

        public void init() {
            timeOut = System.currentTimeMillis() + 2500;
            jewelArmUp = false;
        }

        public void execute(){
            telemetry.addData("Mode:", "Jewel Arm Up");
            //io.jewelArmUp();

            // Keep stepping down until we hit the min value.
            position -= INCREMENT ;
            if (position <= MIN_POS ) {
                position = MIN_POS;
                jewelArmUp = true;
                //rampUp = !rampUp;  // Switch ramp direction
            }

            //io.setJewelArm(position);
        }

        public boolean isFinished(){
            return jewelArmUp || System.currentTimeMillis() >= timeOut;
        }
        public void stop() { io.setDrivePower(0,0, 0, 0); }

}

