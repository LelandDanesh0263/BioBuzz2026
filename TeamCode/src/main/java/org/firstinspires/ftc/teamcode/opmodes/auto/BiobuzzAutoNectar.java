package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.config.RobotStateManager;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

@Autonomous(name = "Biobuzz Auto - Nectar Focus", group = "Autonomous")
public class BiobuzzAutoNectar extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Broadcast telemetry to both Driver Station Hub & FTC Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Pose2d startPose = new Pose2d(12.0, -62.0, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        OuttakeSubsystem outtake = new OuttakeSubsystem(hardwareMap);
        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap);
        TransferSubsystem transfer = new TransferSubsystem(hardwareMap);
        VisionSubsystem vision = new VisionSubsystem(hardwareMap);
        RobotStateManager stateManager = new RobotStateManager(hardwareMap);

        telemetry.addData("Status", "Initialized for Nectar Auto");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // Step 1: Raise outtake to High Target and score preload
        outtake.setTargetHeightInches(RobotConstants.OUTTAKE_HIGH_TARGET_HEIGHT);
        sleep(1200);
        outtake.openClaw();
        sleep(300);
        outtake.closeClaw();
        outtake.setTargetHeightInches(RobotConstants.OUTTAKE_STOW_HEIGHT);

        // Step 2: Deploy intake to collect Nectar
        intake.deployIntakeGround();
        intake.spinIn();
        sleep(1500);
        intake.stopIntake();
        intake.stowIntake();

        // Step 3: Transfer ball
        transfer.transferForward();
        sleep(1000);
        transfer.stopTransfer();

        // Save pose for TeleOp
        stateManager.saveRobotPose(drive.getPose());

        telemetry.addData("Status", "Nectar Autonomous Completed!");
        telemetry.update();
    }
}
