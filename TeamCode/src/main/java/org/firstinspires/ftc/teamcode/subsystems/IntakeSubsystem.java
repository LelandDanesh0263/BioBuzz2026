package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.config.RobotConstants;

public class IntakeSubsystem extends SubsystemBase {

    private final DcMotorEx intakeMotor;
    private final Servo pitchServo;
    private final Servo clawServo;
    private ColorSensor colorSensor;
    private DistanceSensor distanceSensor;

    public enum BallState {
        NONE,
        NECTAR,
        POLLEN
    }

    public IntakeSubsystem(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, RobotConstants.MOTOR_INTAKE);
        pitchServo = hardwareMap.get(Servo.class, RobotConstants.SERVO_INTAKE_PITCH);
        clawServo = hardwareMap.get(Servo.class, RobotConstants.SERVO_INTAKE_CLAW);

        try {
            colorSensor = hardwareMap.get(ColorSensor.class, RobotConstants.SENSOR_COLOR_INTAKE);
            distanceSensor = hardwareMap.get(DistanceSensor.class, RobotConstants.SENSOR_COLOR_INTAKE);
        } catch (Exception e) {
            colorSensor = null;
            distanceSensor = null;
        }

        stowIntake();
    }

    public void spinIn() {
        intakeMotor.setPower(1.0);
    }

    public void spinOut() {
        intakeMotor.setPower(-1.0);
    }

    public void stopIntake() {
        intakeMotor.setPower(0.0);
    }

    public void deployIntakeGround() {
        pitchServo.setPosition(RobotConstants.INTAKE_PITCH_GROUND);
        clawServo.setPosition(RobotConstants.INTAKE_CLAW_OPEN);
    }

    public void stowIntake() {
        pitchServo.setPosition(RobotConstants.INTAKE_PITCH_STOW);
        clawServo.setPosition(RobotConstants.INTAKE_CLAW_CLOSED);
    }

    public void openClaw() {
        clawServo.setPosition(RobotConstants.INTAKE_CLAW_OPEN);
    }

    public void closeClaw() {
        clawServo.setPosition(RobotConstants.INTAKE_CLAW_CLOSED);
    }

    public boolean hasBall() {
        if (distanceSensor == null) return false;
        double distanceMm = distanceSensor.getDistance(DistanceUnit.MM);
        return distanceMm > 0 && distanceMm <= 40.0;
    }

    public BallState detectBallType() {
        if (!hasBall()) return BallState.NONE;
        if (colorSensor == null) return BallState.NECTAR;

        // Yellow detection for Nectar
        int red = colorSensor.red();
        int green = colorSensor.green();
        int blue = colorSensor.blue();

        if (red > 150 && green > 150 && blue < 100) {
            return BallState.NECTAR;
        } else {
            return BallState.POLLEN;
        }
    }
}
