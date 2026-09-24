package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.control.PIDFController;
import org.firstinspires.ftc.teamcode.control.SlewRateLimiter;

public class OuttakeSubsystem extends SubsystemBase {

    private final DcMotorEx slideMotor;
    private final Servo clawServo;
    private final VoltageSensor batteryVoltageSensor;
    private final PIDFController pidf;
    private final SlewRateLimiter rateLimiter;

    private double targetInches = 0.0;

    public OuttakeSubsystem(HardwareMap hardwareMap) {
        slideMotor = hardwareMap.get(DcMotorEx.class, RobotConstants.MOTOR_OUTTAKE_SLIDE);
        clawServo = hardwareMap.get(Servo.class, RobotConstants.SERVO_OUTTAKE_CLAW);
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pidf = new PIDFController(
                RobotConstants.OUTTAKE_KP,
                RobotConstants.OUTTAKE_KI,
                RobotConstants.OUTTAKE_KD,
                RobotConstants.OUTTAKE_KF,
                RobotConstants.OUTTAKE_KG,
                RobotConstants.OUTTAKE_KS
        );

        rateLimiter = new SlewRateLimiter(RobotConstants.OUTTAKE_MAX_VELOCITY_INCHES_PER_SEC, 0);
        closeClaw();
    }

    public void setTargetHeightInches(double inches) {
        this.targetInches = Math.max(0.0, Math.min(inches, RobotConstants.OUTTAKE_MAX_HEIGHT_INCHES));
    }

    public void update() {
        // Dynamic gain updating from FTC Dashboard @Config
        pidf.setCoefficients(
                RobotConstants.OUTTAKE_KP,
                RobotConstants.OUTTAKE_KI,
                RobotConstants.OUTTAKE_KD,
                RobotConstants.OUTTAKE_KF,
                RobotConstants.OUTTAKE_KG,
                RobotConstants.OUTTAKE_KS
        );

        double currentInches = getCurrentHeightInches();
        double profileTarget = rateLimiter.update(targetInches);

        double rawPower = pidf.calculate(profileTarget, currentInches);

        double batteryVoltage = batteryVoltageSensor.getVoltage();
        double voltageScaledPower = rawPower * (12.0 / (batteryVoltage > 0 ? batteryVoltage : 12.0));
        double clampedPower = Math.max(-1.0, Math.min(1.0, voltageScaledPower));

        slideMotor.setPower(clampedPower);
    }

    @Override
    public void periodic() {
        update();
    }

    public void openClaw() {
        clawServo.setPosition(RobotConstants.OUTTAKE_CLAW_OPEN);
    }

    public void closeClaw() {
        clawServo.setPosition(RobotConstants.OUTTAKE_CLAW_CLOSED);
    }

    public double getCurrentHeightInches() {
        return slideMotor.getCurrentPosition() / RobotConstants.OUTTAKE_TICKS_PER_INCH;
    }

    public double getTargetHeightInches() {
        return targetInches;
    }

    public boolean isAtTarget() {
        return Math.abs(targetInches - getCurrentHeightInches()) <= 0.5;
    }
}
