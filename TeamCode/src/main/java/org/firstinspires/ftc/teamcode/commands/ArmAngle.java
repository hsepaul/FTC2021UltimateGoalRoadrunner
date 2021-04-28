package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.Math.round;

public class ArmAngle {
    private static long shooterAngle;
    private static AnalogInput potentiometerAA;
    private static DcMotorEx motorAA;
    private static ElapsedTime timeout;
    public static boolean isFinished;
    public static void AngleArm(double angle, LinearOpMode op) {
        timeout = new ElapsedTime();
        potentiometerAA = op.hardwareMap.get(AnalogInput.class, "potentiometerAA");
        motorAA = op.hardwareMap.get(DcMotorEx.class, "motorAA");
        shooterAngle = round((10 * (138.2488479 * potentiometerAA.getVoltage()) - 84.0829493)) / 10;
        isFinished = false;
        timeout.reset();
        while (timeout.seconds() < 2 && isFinished == false){
            if(shooterAngle < angle && shooterAngle > angle - 2){
                motorAA.setPower(-.3);
                op.telemetry.addData("moving up slow", shooterAngle);
            }
            if(shooterAngle < angle && !(shooterAngle > angle - 2)){
                motorAA.setPower(-1);
                op.telemetry.addData("moving up fast", shooterAngle);
            }
            if(shooterAngle > angle && shooterAngle < angle + 2){
                motorAA.setPower(.3);
                op.telemetry.addData("moving down slow", shooterAngle);
            }
            if(shooterAngle > angle && !(shooterAngle < angle + 2)){
                motorAA.setPower(1);
                op.telemetry.addData("moving down fast", shooterAngle);
            }
            if(shooterAngle == angle){
                isFinished = true;
            }
            op.telemetry.addData("shooter angle", shooterAngle);
            op.telemetry.addData("Angle no round", 138.2488479 * potentiometerAA.getVoltage() - 84.0829493);
            shooterAngle = round((10 * ((138.2488479 * potentiometerAA.getVoltage()) - 84.0829493))) / 10;
            op.telemetry.update();

        }
        isFinished = true;
    }
    public boolean isDone() {
        return isFinished;
    }
}
