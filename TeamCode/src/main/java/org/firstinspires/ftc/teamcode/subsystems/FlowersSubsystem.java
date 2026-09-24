package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.control.PIDFController;

public class FlowersSubsystem extends SubsystemBase {

    private final DcMotorEx flowersMotor;
    private final Servo pusherServo;
    private final PIDFController pidf;

    private double targetTicks = 0.0;

    public FlowersSubsystem(HardwareMap hardwareMap) {
        DcMotorEx motorTemp;
        Servo servoTemp;

        try {
            motorTemp = hardwareMap.get(DcMotorEx.class, RobotConstants.MOTOR_FLOWERS);
            motorTemp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorTemp.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorTemp.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } catch (Exception e) {
            motorTemp = null;
        }

        try {
            servoTemp = hardwareMap.get(Servo.class, RobotConstants.SERVO_FLOWERS_PUSHER);
        } catch (Exception e) {
            servoTemp = null;
        }

        this.flowersMotor = motorTemp;
        this.pusherServo = servoTemp;

        pidf = new PIDFController(
                RobotConstants.FLOWERS_KP,
                RobotConstants.FLOWERS_KI,
                RobotConstants.FLOWERS_KD,
                RobotConstants.FLOWERS_KF,
                RobotConstants.FLOWERS_KG,
                0.0
        );

        retractPusher();
        stow();
    }

    public void setTargetTicks(double ticks) {
        this.targetTicks = ticks;
    }

    public void stow() {
        setTargetTicks(RobotConstants.FLOWERS_STOW_POSITION);
        retractPusher();
    }

    public void deployForFlowersScore() {
        setTargetTicks(RobotConstants.FLOWERS_SCORE_POSITION);
    }

    public void extendPusher() {
        if (pusherServo != null) {
            pusherServo.setPosition(RobotConstants.FLOWERS_PUSHER_EXTEND);
        }
    }

    public void retractPusher() {
        if (pusherServo != null) {
            pusherServo.setPosition(RobotConstants.FLOWERS_PUSHER_RETRACT);
        }
    }

    public void update() {
        if (flowersMotor == null) return;

        pidf.setCoefficients(
                RobotConstants.FLOWERS_KP,
                RobotConstants.FLOWERS_KI,
                RobotConstants.FLOWERS_KD,
                RobotConstants.FLOWERS_KF,
                RobotConstants.FLOWERS_KG,
                0.0
        );

        double currentTicks = flowersMotor.getCurrentPosition();
        double power = pidf.calculate(targetTicks, currentTicks);
        double clampedPower = Math.max(-1.0, Math.min(1.0, power));

        flowersMotor.setPower(clampedPower);
    }

    @Override
    public void periodic() {
        update();
    }

    public double getCurrentTicks() {
        return flowersMotor != null ? flowersMotor.getCurrentPosition() : 0.0;
    }

    public boolean isAtTarget() {
        return Math.abs(targetTicks - getCurrentTicks()) <= 15.0;
    }
}
