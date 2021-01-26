package org.firstinspires.ftc.teamcode.commands;


/**
 * Created by HPaul on 10/22/2017.
 */

public class CalibrationSkystoneCommand extends BasicCommand {
    long timeOut;
    long wakeupTime;
    boolean calibrationComplete = false;

    boolean calibrationArmExtenderDone = false;
    boolean calibrationArmAngleDone = false;


    public CalibrationSkystoneCommand(long timeOut){ this.timeOut = timeOut; }

    public void init() {
        wakeupTime = System.currentTimeMillis() + timeOut;
        //timeOut = System.currentTimeMillis() + 8000;
        io.rightHookMidDown();
        io.leftHookUp();

        //io.gripperRotateStowed();
        io.resetDriveEncoders();

        // Tell the driver that initialization is complete.
        if (io.touchArmExtender.getState() == false) {
            telemetry.addData("Calibration of Arm Extender", "Not Required");
        } else {
            telemetry.addData("Calibration of Arm Extender", "Required");
        }

        if (io.touchArmAngle.getState() == false) {
            telemetry.addData("Calibration of Arm Angle", "Not Required");
        } else {
            telemetry.addData("Calibration of Arm Angle", "Required");
        }


        calibrationComplete = false;
    }

    public void execute(){
        //io.gripperRotateStowed();

        if ((io.touchArmExtender.getState() == true) && !calibrationArmExtenderDone) {
            io.armExtenderMotor.setPower(.5);
            telemetry.addData("Calibration of Arm Extender", "In Process");
        } else{
            io.armExtenderMotor.setPower(0);
            io.resetDriveEncoders();
            telemetry.addData("Status", "Resetting Encoders");
            telemetry.addData("Calibration of Arm Extender", "Completed");
            calibrationArmExtenderDone = true;
        }

        if ((io.touchArmAngle.getState() == true) && (!calibrationArmAngleDone)){
            io.armAngleMotor.setPower(.5);
            telemetry.addData("Calibration of Arm Angle", "In Process");
        } else {
            io.armAngleMotor.setPower(0);
            io.resetDriveEncoders();
            telemetry.addData("Status", "Resetting Encoders");
            telemetry.addData("Calibration of Arm Angle", "Completed");
            calibrationArmAngleDone = true;
        }

        if (calibrationArmExtenderDone && calibrationArmAngleDone) {
            calibrationComplete = true;
        }
    }

    public boolean isFinished(){
        return calibrationComplete || System.currentTimeMillis() >= wakeupTime;
    }
    public void stop() {
        //io.hookStop();
        //io.setDrivePower(0,0);
        //io.forkLiftMotor.setPower(0);
    }

}

