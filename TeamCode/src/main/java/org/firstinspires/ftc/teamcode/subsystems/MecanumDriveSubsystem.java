package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.control.PIDController;

public class MecanumDriveSubsystem extends SubsystemBase {

    private final DcMotorEx frontLeft;
    private final DcMotorEx frontRight;
    private final DcMotorEx backLeft;
    private final DcMotorEx backRight;
    private final IMU imu;

    private final PIDController headingController;

    public enum SpeedMode {
        TURBO(1.0),
        NORMAL(0.7),
        SLOW(0.35);

        public final double multiplier;
        SpeedMode(double multiplier) {
            this.multiplier = multiplier;
        }
    }

    private SpeedMode currentSpeedMode = SpeedMode.NORMAL;
    private boolean fieldCentric = true;
    private double headingOffsetRad = 0.0;

    public MecanumDriveSubsystem(HardwareMap hardwareMap) {
        frontLeft = hardwareMap.get(DcMotorEx.class, RobotConstants.MOTOR_FRONT_LEFT);
        frontRight = hardwareMap.get(DcMotorEx.class, RobotConstants.MOTOR_FRONT_RIGHT);
        backLeft = hardwareMap.get(DcMotorEx.class, RobotConstants.MOTOR_BACK_LEFT);
        backRight = hardwareMap.get(DcMotorEx.class, RobotConstants.MOTOR_BACK_RIGHT);

        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        backLeft.setDirection(DcMotorEx.Direction.REVERSE);
        frontRight.setDirection(DcMotorEx.Direction.FORWARD);
        backRight.setDirection(DcMotorEx.Direction.FORWARD);

        DcMotorEx[] motors = {frontLeft, frontRight, backLeft, backRight};
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );
        imu.initialize(parameters);

        headingController = new PIDController(
                RobotConstants.HEADING_KP,
                RobotConstants.HEADING_KI,
                RobotConstants.HEADING_KD
        );
    }

    public void drive(double strafe, double forward, double turn) {
        headingController.setGains(RobotConstants.HEADING_KP, RobotConstants.HEADING_KI, RobotConstants.HEADING_KD);

        double driveForward = forward * currentSpeedMode.multiplier;
        double driveStrafe = strafe * currentSpeedMode.multiplier;
        double driveTurn = turn * currentSpeedMode.multiplier;

        if (fieldCentric) {
            double botHeading = getHeadingRadians();
            double rotX = driveStrafe * Math.cos(-botHeading) - driveForward * Math.sin(-botHeading);
            double rotY = driveStrafe * Math.sin(-botHeading) + driveForward * Math.cos(-botHeading);

            driveStrafe = rotX;
            driveForward = rotY;
        }

        double denominator = Math.max(Math.abs(driveForward) + Math.abs(driveStrafe) + Math.abs(driveTurn), 1.0);
        double frontLeftPower = (driveForward + driveStrafe + driveTurn) / denominator;
        double backLeftPower = (driveForward - driveStrafe + driveTurn) / denominator;
        double frontRightPower = (driveForward - driveStrafe - driveTurn) / denominator;
        double backRightPower = (driveForward + driveStrafe - driveTurn) / denominator;

        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);
    }

    public void driveHeadingLock(double strafe, double forward, double targetHeadingRad) {
        double currentHeading = getHeadingRadians();
        double error = AngleUnit.normalizeRadians(targetHeadingRad - currentHeading);
        double turnOutput = headingController.calculate(0, -error);
        drive(strafe, forward, turnOutput);
    }

    public double getHeadingRadians() {
        return AngleUnit.normalizeRadians(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) - headingOffsetRad);
    }

    public void resetHeading() {
        headingOffsetRad = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        imu.resetYaw();
    }

    public void setSpeedMode(SpeedMode speedMode) {
        this.currentSpeedMode = speedMode;
    }

    public void setFieldCentric(boolean enabled) {
        this.fieldCentric = enabled;
    }

    public boolean isFieldCentric() {
        return fieldCentric;
    }

    public void stop() {
        drive(0, 0, 0);
    }
}
