package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDFController {

    private double Kp, Ki, Kd, Kf;
    private double kG = 0; // Gravity feedforward constant
    private double kS = 0; // Static friction feedforward
    private double kV = 0; // Velocity feedforward
    private double integralSum = 0;
    private double lastError = 0;
    private double lastFilterDerivative = 0;
    private double tolerance = 0.05;
    private final ElapsedTime timer = new ElapsedTime();

    public PIDFController(double Kp, double Ki, double Kd, double Kf) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.Kf = Kf;
        timer.reset();
    }

    public PIDFController(double Kp, double Ki, double Kd, double Kf, double kG, double kS) {
        this(Kp, Ki, Kd, Kf);
        this.kG = kG;
        this.kS = kS;
    }

    public void setCoefficients(double Kp, double Ki, double Kd, double Kf, double kG, double kS) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.Kf = Kf;
        this.kG = kG;
        this.kS = kS;
    }

    public void setTolerance(double tolerance) {
        this.tolerance = tolerance;
    }

    public double calculate(double target, double current) {
        double dt = timer.seconds();
        timer.reset();

        if (dt <= 0 || dt > 0.5) {
            dt = 0.02;
        }

        double error = target - current;

        // Anti-windup
        integralSum += error * dt;
        integralSum = Math.max(-100.0, Math.min(100.0, integralSum));

        // Derivative term with low pass filter
        double rawDerivative = (error - lastError) / dt;
        double derivative = (0.8 * lastFilterDerivative) + (0.2 * rawDerivative);

        lastError = error;
        lastFilterDerivative = derivative;

        double pidOutput = (Kp * error) + (Ki * integralSum) + (Kd * derivative);

        // Feedforward terms: Kf * target + kG (gravity compensation) + kS * sign(error)
        double ffOutput = (Kf * target) + kG + (Math.signum(error) * kS);

        return pidOutput + ffOutput;
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
}
