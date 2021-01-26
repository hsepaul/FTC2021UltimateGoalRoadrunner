package org.firstinspires.ftc.teamcode.commands;


/**
 * Created by HPaul on 10/22/2017.
 */

public class SetGyroOffset extends BasicCommand {

        public SetGyroOffset(){
        }

        public void init() {
            //io.setGyroOffset();
        }

        public void execute(){
            telemetry.addData("Mode:", "Set Gyro Offset");
        }

        public boolean isFinished(){
            return true;
        }
        public void stop() {
            io.setDrivePower(0,0, 0, 0);
        }

}

