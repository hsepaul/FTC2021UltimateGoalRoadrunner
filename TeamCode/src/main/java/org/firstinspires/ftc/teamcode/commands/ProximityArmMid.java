package org.firstinspires.ftc.teamcode.commands;


/**
 * Created by HPaul on 10/22/2017.
 */

public class ProximityArmMid extends BasicCommand {

    boolean proximityArmMid = false;
    long timeOut;
    int loops = 0;
    static final double INCREMENT   = 0.05; //was 0.02    // amount to slew servo each CYCLE_MS cycle
    //static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_POS     =  0.40;     // Maximum rotational position
    static final double MIN_POS     =  0.00;     // Minimum rotational position
    double  position = MIN_POS;

        public ProximityArmMid(){

        }

        public void init() {
            timeOut = System.currentTimeMillis() + 2500;
            proximityArmMid = false;
        }

        public void execute(){
            telemetry.addData("Mode:", "Proximity Arm Middle");
            //io.jewelArmDown();

            // Keep stepping up until we hit the max value.
            position += INCREMENT ;
            if (position >= MAX_POS ) {
                position = MAX_POS;
                proximityArmMid = true;
                //rampUp = !rampUp;   // Switch ramp direction
            }

            //io.setProximityArm(position);
        }

        public boolean isFinished(){
            return proximityArmMid || System.currentTimeMillis() >= timeOut;
        }
        public void stop() {
            io.setDrivePower(0,0, 0, 0);
        }

}

