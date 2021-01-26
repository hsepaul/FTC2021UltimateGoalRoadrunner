package org.firstinspires.ftc.teamcode.utilities;

/**
 * Created by David Austin on 10/27/2016.
 */

public class HSPID {
    double Kp,Ki,Kd;
    double totalError = 0;
    double lastError = 0;
    double alpha = .8;
    double target;

    public HSPID(double Kp, double Ki, double Kd){
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;

    }
    public void setTarget(double target){
        this.target = target;
    }

    public double getCorrection(double current){
        double error = target - current;
        double changeError = error - lastError;
        if (lastError * error <= 0) totalError = 0;
        else totalError = alpha * totalError + error;
        lastError = error;
        return (Kp*error+Ki*totalError+Kd*changeError);

    }
    public void setGains(double Kp, double Ki, double Kd) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
    }

}
