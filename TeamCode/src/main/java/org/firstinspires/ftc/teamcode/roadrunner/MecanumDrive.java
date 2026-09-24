package org.firstinspires.ftc.teamcode.roadrunner;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.teamcode.config.RobotConstants;

@Config
public class MecanumDrive {

    public final DcMotorEx frontLeft;
    public final DcMotorEx frontRight;
    public final DcMotorEx backLeft;
    public final DcMotorEx backRight;
    public final IMU imu;

    public final MecanumKinematics kinematics;
    public Pose2d pose;

    public MecanumDrive(HardwareMap hardwareMap, Pose2d initialPose) {
        this.pose = initialPose;

        kinematics = new MecanumKinematics(RobotConstants.TRACK_WIDTH_INCHES);

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
    }

    public void setDrivePowers(PoseVelocity2d powers) {
        double vx = powers.linearVel.x;
        double vy = powers.linearVel.y;
        double omega = powers.angVel;

        double fl = vx - vy - omega;
        double bl = vx + vy - omega;
        double fr = vx + vy + omega;
        double br = vx - vy + omega;

        double max = Math.max(1.0, Math.max(Math.abs(fl), Math.max(Math.abs(bl), Math.max(Math.abs(fr), Math.abs(br)))));

        frontLeft.setPower(fl / max);
        backLeft.setPower(bl / max);
        frontRight.setPower(fr / max);
        backRight.setPower(br / max);
    }

    public Pose2d getPose() {
        return pose;
    }

    public void setPose(Pose2d newPose) {
        this.pose = newPose;
    }
}
