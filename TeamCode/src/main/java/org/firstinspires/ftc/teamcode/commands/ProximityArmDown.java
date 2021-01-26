package org.firstinspires.ftc.teamcode.commands;


/**
 * Created by HPaul on 10/22/2017.
 */

public class ProximityArmDown extends BasicCommand {

    boolean proximityArmDown = false;
    long timeOut;
    int loops = 0;
    static final double INCREMENT   = 0.05; //was 0.02     // amount to slew servo each CYCLE_MS cycle
    //static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_POS     =  .85; //was 1.00     // Maximum rotational position
    static final double MIN_POS     =  0.40;     // Minimum rotational position
    double  position = MIN_POS;

        public ProximityArmDown(){

        }

        public void init() {
            timeOut = System.currentTimeMillis() + 2500;
            proximityArmDown = false;
        }

        public void execute(){
            telemetry.addData("Mode:", "Proximity Arm Down");
            //io.jewelArmDown();

            // Keep stepping up until we hit the max value.
            position += INCREMENT ;
            if (position >= MAX_POS ) {
                position = MAX_POS;
                proximityArmDown = true;
                //rampUp = !rampUp;   // Switch ramp direction
            }

            //io.setProximityArm(position);
        }

        public boolean isFinished(){
            return proximityArmDown || System.currentTimeMillis() >= timeOut;
        }
        public void stop() {
            io.setDrivePower(0,0, 0, 0);
        }

}

