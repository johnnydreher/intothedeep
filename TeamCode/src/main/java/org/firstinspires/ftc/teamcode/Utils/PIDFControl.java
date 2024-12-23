package org.firstinspires.ftc.teamcode.Utils;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDFControl {
    ElapsedTime timer = new ElapsedTime();
    private double lastError;
    double setPoint;
    double integralSum;
    double lastRead;
    double kp;
    double ki;
    double kd;
    double kf;
    public PIDFControl(double setPoint, double kp, double ki, double kd, double kf){
        this.kd = kd;
        this.ki = ki;
        this.kp = kp;
        this.kf =kf;
        this.setPoint = setPoint;
        integralSum = 0;
        lastError = 0;
        timer.reset();
        lastRead = 0;
    }
    public void updateSetpoint(double setPoint, double kp, double ki, double kd, double kf){
        this.kd = kd;
        this.ki = ki;
        this.kp = kp;
        this.kf =kf;
        this.setPoint = setPoint;
        integralSum = 0;
        lastError = 0;
        timer.reset();
        lastRead = 0;
    }
    public void updateSetpoint(double setPoint){

        this.setPoint = setPoint;

        timer.reset();
    }
    public double get(double read) {
        double error = setPoint - read;
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / (timer.seconds());
        lastError = error;
        timer.reset();
        lastRead = read;
        return  (error * kp) + (derivative * kd) + (integralSum * ki) + kf;
    }
    public double angleWrap(double radians){
        while(radians > Math.PI){
            radians -= 2 * Math.PI;
        }
        while(radians < -Math.PI){
            radians += 2 * Math.PI;
        }
        return radians;
    }
    public boolean atSetpoint(){
        double setPointMin = Math.abs(setPoint)*0.99;
        double setPointMax = Math.abs(setPoint)*1.01;
        return (Math.abs(lastRead)>=setPointMin && Math.abs(lastRead)<=setPointMax);
    }
}
