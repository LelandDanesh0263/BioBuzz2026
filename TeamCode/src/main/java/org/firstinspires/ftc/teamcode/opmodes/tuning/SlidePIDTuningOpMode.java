package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.config.RobotStateManager;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;

@TeleOp(name = "Tuning - Outtake Slide PIDF Step Response", group = "Tuning")
public class SlidePIDTuningOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Broadcast telemetry to both Driver Station Hub & FTC Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        RobotStateManager stateManager = new RobotStateManager(hardwareMap);
        stateManager.loadTunedPidGains(); // Load previously saved gains

        OuttakeSubsystem outtake = new OuttakeSubsystem(hardwareMap);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        ElapsedTime timer = new ElapsedTime();

        double lowSetpoint = 2.0;
        double highSetpoint = 22.0;
        boolean isHighTarget = false;

        telemetry.addData("Status", "Outtake Slide Step Response Tuning Ready");
        telemetry.addData("Instructions", "Toggle 'SAVE_PID_VALUES_TO_ROBOT = true' on Dashboard to save tuned numbers to robot memory!");
        telemetry.update();

        waitForStart();
        timer.reset();

        while (opModeIsActive()) {
            stateManager.checkAndHandleDashboardSave();

            if (timer.seconds() >= 2.5) {
                isHighTarget = !isHighTarget;
                timer.reset();
            }

            double currentTarget = isHighTarget ? highSetpoint : lowSetpoint;
            outtake.setTargetHeightInches(currentTarget);
            outtake.update();

            double currentPos = outtake.getCurrentHeightInches();
            double error = currentTarget - currentPos;

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Target (in)", currentTarget);
            packet.put("Position (in)", currentPos);
            packet.put("Error (in)", error);
            packet.put("OUTTAKE_KP", RobotConstants.OUTTAKE_KP);
            packet.put("OUTTAKE_KI", RobotConstants.OUTTAKE_KI);
            packet.put("OUTTAKE_KD", RobotConstants.OUTTAKE_KD);
            packet.put("OUTTAKE_KG", RobotConstants.OUTTAKE_KG);

            dashboard.sendTelemetryPacket(packet);

            telemetry.addData("Target Height", "%.2f in", currentTarget);
            telemetry.addData("Current Height", "%.2f in", currentPos);
            telemetry.addData("Error", "%.2f in", error);
            telemetry.addData("Gains", "Kp=%.5f, Ki=%.5f, Kd=%.5f, kG=%.4f",
                    RobotConstants.OUTTAKE_KP, RobotConstants.OUTTAKE_KI, RobotConstants.OUTTAKE_KD, RobotConstants.OUTTAKE_KG);
            telemetry.update();
        }
    }
}
