package org.firstinspires.ftc.teamcode.commands;


/**
 * Created by HPaul on 10/22/2017.
 */

public class SetIMUOffset extends BasicCommand {

        public SetIMUOffset(){
        }

        public void init() {
            io.setIMUOffset();
        }

        public void execute(){
            telemetry.addData("Mode:", "Set IMU Offset");
        }

        public boolean isFinished(){
            return true;
        }
        public void stop() {
            io.setDrivePower(0,0, 0, 0);
        }

}

