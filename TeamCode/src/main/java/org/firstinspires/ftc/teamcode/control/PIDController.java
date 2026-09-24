package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {

    private double Kp, Ki, Kd;
    private double integralSum = 0;
    private double lastError = 0;
    private double lastFilterDerivative = 0;
    private double tolerance = 0.05;
    private final ElapsedTime timer = new ElapsedTime();

    public PIDController(double Kp, double Ki, double Kd) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        timer.reset();
    }

    public void setGains(double Kp, double Ki, double Kd) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
    }

    public void setTolerance(double tolerance) {
        this.tolerance = tolerance;
    }

    public double calculate(double target, double current) {
        double dt = timer.seconds();
        timer.reset();

        if (dt <= 0 || dt > 0.5) {
            dt = 0.02; // Default fallback for first run or pause
        }

        double error = target - current;

        // Anti-windup clamping
        integralSum += error * dt;
        integralSum = Math.max(-100.0, Math.min(100.0, integralSum));

        // Low-pass filtered derivative term (alpha = 0.8)
        double rawDerivative = (error - lastError) / dt;
        double derivative = (0.8 * lastFilterDerivative) + (0.2 * rawDerivative);

        lastError = error;
        lastFilterDerivative = derivative;

        return (Kp * error) + (Ki * integralSum) + (Kd * derivative);
    }

    public boolean atSetpoint(double target, double current) {
        return Math.abs(target - current) <= tolerance;
    }

    public void reset() {
        integralSum = 0;
        lastError = 0;
        lastFilterDerivative = 0;
        timer.reset();
    }

    public double getKp() { return Kp; }
    public double getKi() { return Ki; }
    public double getKd() { return Kd; }
}
