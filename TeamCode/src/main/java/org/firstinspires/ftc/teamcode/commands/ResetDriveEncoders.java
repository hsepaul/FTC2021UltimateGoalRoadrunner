package org.firstinspires.ftc.teamcode.commands;


/**
 * Created by HPaul on 10/22/2017.
 */

public class ResetDriveEncoders extends BasicCommand {

        public ResetDriveEncoders(){
        }

        public void init() {
            io.resetDriveEncoders();
        }

        public void execute(){
            telemetry.addData("Mode:", "Reset Drive Encoders");
            //telemetry.addData("x after reset: ",io.getX());
            //telemetry.addData("y after reset: ",io.getY());
            //telemetry.addData("center after reset: ",io.getSidewaysDistance());
        }

        public boolean isFinished(){
            //telemetry.addData("Mode:", "Reset Drive Encoders");
            //telemetry.addData("x after reset: ",io.getX());
            //telemetry.addData("y after reset: ",io.getY());
            //telemetry.addData("center after reset get sideways distance: ",io.getSidewaysDistance());
            //telemetry.addData("center after reset get odometer center encoder: ",io.getOdometerCenterEncoder());
            //telemetry.addData("center after reset back right motor encoder: ",io.backRightMotor.getCurrentPosition());
            return true;
        }
        public void stop() {
            io.setDrivePower(0,0, 0, 0);
        }

}

