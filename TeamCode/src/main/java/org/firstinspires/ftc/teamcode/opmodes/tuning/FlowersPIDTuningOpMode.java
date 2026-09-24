package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.config.RobotStateManager;
import org.firstinspires.ftc.teamcode.subsystems.FlowersSubsystem;

@TeleOp(name = "Tuning - Flowers Mechanism PIDF Step Response", group = "Tuning")
public class FlowersPIDTuningOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Broadcast telemetry to both Driver Station Hub & FTC Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        RobotStateManager stateManager = new RobotStateManager(hardwareMap);
        stateManager.loadTunedPidGains(); // Load previously saved gains

        FlowersSubsystem flowers = new FlowersSubsystem(hardwareMap);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        ElapsedTime timer = new ElapsedTime();

        double stowSetpoint = RobotConstants.FLOWERS_STOW_POSITION;
        double scoreSetpoint = RobotConstants.FLOWERS_SCORE_POSITION;
        boolean isScoreTarget = false;

        telemetry.addData("Status", "Flowers Mechanism Step Response Tuning Ready");
        telemetry.addData("Instructions", "Toggle 'SAVE_PID_VALUES_TO_ROBOT = true' on Dashboard to save tuned numbers to robot memory!");
        telemetry.update();

        waitForStart();
        timer.reset();

        while (opModeIsActive()) {
            stateManager.checkAndHandleDashboardSave();

            if (timer.seconds() >= 2.5) {
                isScoreTarget = !isScoreTarget;
                timer.reset();
            }

            double currentTarget = isScoreTarget ? scoreSetpoint : stowSetpoint;
            flowers.setTargetTicks(currentTarget);
            flowers.update();

            double currentPos = flowers.getCurrentTicks();
            double error = currentTarget - currentPos;

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Target (ticks)", currentTarget);
            packet.put("Position (ticks)", currentPos);
            packet.put("Error (ticks)", error);
            packet.put("FLOWERS_KP", RobotConstants.FLOWERS_KP);
            packet.put("FLOWERS_KI", RobotConstants.FLOWERS_KI);
            packet.put("FLOWERS_KD", RobotConstants.FLOWERS_KD);
            packet.put("FLOWERS_KG", RobotConstants.FLOWERS_KG);

            dashboard.sendTelemetryPacket(packet);

            telemetry.addData("Target Ticks", "%.1f", currentTarget);
            telemetry.addData("Current Ticks", "%.1f", currentPos);
            telemetry.addData("Error Ticks", "%.1f", error);
            telemetry.addData("Gains", "Kp=%.5f, Ki=%.5f, Kd=%.5f, kG=%.4f",
                    RobotConstants.FLOWERS_KP, RobotConstants.FLOWERS_KI, RobotConstants.FLOWERS_KD, RobotConstants.FLOWERS_KG);
            telemetry.update();
        }
    }
}
