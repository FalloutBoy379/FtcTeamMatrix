package org.firstinspires.ftc.teamcode.drive;

public class PIDFController {
    private double kp;  // Proportional gain
    private double ki;  // Integral gain
    private double kd;  // Derivative gain
    private double kf;  // Feedforward gain
    private double target;  // Target setpoint
    private double integralSum;  // Integral sum
    private double previousError;  // Previous error

    public double tolerance = 0;

    public PIDFController(double kp, double ki, double kd, double kf) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.kf = kf;
        this.integralSum = 0.0;
        this.previousError = 0.0;
    }

    public void setTolerance(double tolerance1){
        tolerance = tolerance1;
    }

    public void setCoeffs(double Kp, double Ki, double Kd, double Kf){
        this.kp = Kp;
        this.kd = Kd;
        this.ki = Ki;
        this.kf = Kf;
    }

    public double calculate(double currentValue, double targetValue) {
        this.target = targetValue;
        double error = target - currentValue;
        integralSum += error;
        double derivative = (error - previousError);

        double output = kp * error + ki * integralSum + kd * derivative + kf * target;
        previousError = error;

        return output;
    }

    public boolean isAtTarget(){
        if(Math.abs(previousError) <= tolerance){
            return true;
        }
        else{
            return false;
        }
    }
}
