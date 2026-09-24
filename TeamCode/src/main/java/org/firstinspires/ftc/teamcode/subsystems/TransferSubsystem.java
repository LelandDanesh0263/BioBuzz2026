package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.config.RobotConstants;

public class TransferSubsystem extends SubsystemBase {

    private final DcMotorEx transferMotor;
    private final Servo gateServo;

    private boolean enabled = true;

    public TransferSubsystem(HardwareMap hardwareMap) {
        DcMotorEx motorTemp;
        Servo gateTemp;

        try {
            motorTemp = hardwareMap.get(DcMotorEx.class, RobotConstants.MOTOR_TRANSFER);
        } catch (Exception e) {
            motorTemp = null;
        }

        try {
            gateTemp = hardwareMap.get(Servo.class, RobotConstants.SERVO_TRANSFER_GATE);
        } catch (Exception e) {
            gateTemp = null;
        }

        this.transferMotor = motorTemp;
        this.gateServo = gateTemp;

        closeGate();
    }

    public void transferForward() {
        if (!enabled) return;
        if (transferMotor != null) transferMotor.setPower(0.8);
        openGate();
    }

    public void transferReverse() {
        if (!enabled) return;
        if (transferMotor != null) transferMotor.setPower(-0.8);
        openGate();
    }

    public void stopTransfer() {
        if (transferMotor != null) transferMotor.setPower(0.0);
        closeGate();
    }

    public void openGate() {
        if (!enabled) return;
        if (gateServo != null) gateServo.setPosition(RobotConstants.TRANSFER_GATE_OPEN);
    }

    public void closeGate() {
        if (gateServo != null) gateServo.setPosition(RobotConstants.TRANSFER_GATE_CLOSED);
    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
        if (!enabled) stopTransfer();
    }

    public boolean isEnabled() {
        return enabled;
    }
}
