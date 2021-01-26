package org.firstinspires.ftc.teamcode.commands;


/**
 * Created by HPaul on 10/22/2017.
 */

public class ProximityArmUp extends BasicCommand {

    boolean proximityArmUp = false;
    long timeOut;
    int loops = 0;
    static final double INCREMENT   = 0.05;  //was 0.02  // amount to slew servo each CYCLE_MS cycle
    //static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_POS     =  .85; //was 1.00    // Maximum rotational position
    static final double MIN_POS     =  0.00;     // Minimum rotational position
    double  position = MAX_POS;


        public ProximityArmUp(){

        }

        public void init() {
            timeOut = System.currentTimeMillis() + 2500;
            proximityArmUp = false;
        }

        public void execute(){
            telemetry.addData("Mode:", "Proximity Arm Up");
            //io.jewelArmUp();

            // Keep stepping down until we hit the min value.
            position -= INCREMENT ;
            if (position <= MIN_POS ) {
                position = MIN_POS;
                proximityArmUp = true;
                //rampUp = !rampUp;  // Switch ramp direction
            }

            //io.setProximityArm(position);
        }

        public boolean isFinished(){ return proximityArmUp || System.currentTimeMillis() >= timeOut; }
        public void stop() { io.setDrivePower(0,0, 0, 0); }

}

