package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.util.ElapsedTime;

public class SlewRateLimiter {

    private double maxRate;
    private double lastValue;
    private final ElapsedTime timer = new ElapsedTime();

    public SlewRateLimiter(double maxRateUnitsPerSec, double initialValue) {
        this.maxRate = maxRateUnitsPerSec;
        this.lastValue = initialValue;
        timer.reset();
    }

    public void setMaxRate(double maxRateUnitsPerSec) {
        this.maxRate = maxRateUnitsPerSec;
    }

    public double update(double targetValue) {
        double dt = timer.seconds();
        timer.reset();

        if (dt <= 0 || dt > 0.5) {
            dt = 0.02;
        }

        double delta = targetValue - lastValue;
        double maxChange = maxRate * dt;

        if (Math.abs(delta) > maxChange) {
            lastValue += Math.signum(delta) * maxChange;
        } else {
            lastValue = targetValue;
        }

        return lastValue;
    }

    public void reset(double value) {
        this.lastValue = value;
        timer.reset();
    }
}
