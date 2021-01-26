package org.firstinspires.ftc.teamcode.commands;


import static org.firstinspires.ftc.teamcode.utilities.Sleep.sleep;

/**
 * Created by HPaul on 10/22/2017.
 */

public class ResetGyro extends BasicCommand {

        public ResetGyro(){
        }

        public void init() {
            io.calibrateGyroandIMU();
            sleep(1000);
        }

        public void execute(){
            /*if (io.gyro.isCalibrating())  {
                telemetry.addData(">", "Gyro Calibrating. Do Not Move!");
            } else {
                telemetry.addData(">", "Gyro Calibrated.");
            }*/
        }

        public boolean isFinished(){
            /*if (!io.gyro.isCalibrating()){
                telemetry.addData(">", "Gyro Calibrated.");
                io.setGyroOffset();
                return true;
            } else {
                return false;
            }*/
            return true;
        }
        public void stop() {
            io.setDrivePower(0,0, 0, 0);
        }

}

