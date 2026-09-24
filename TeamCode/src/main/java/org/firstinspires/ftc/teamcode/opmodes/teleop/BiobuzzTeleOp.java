package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.AlignToAprilTagCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.FlowersScoreCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.OuttakeCommand;
import org.firstinspires.ftc.teamcode.commands.ScoreSequenceCommand;
import org.firstinspires.ftc.teamcode.commands.TransferCommand;
import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.config.RobotStateManager;
import org.firstinspires.ftc.teamcode.subsystems.FlowersSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.vision.BallVisionPipeline;

@TeleOp(name = "Biobuzz TeleOp", group = "TeleOp")
public class BiobuzzTeleOp extends CommandOpMode {

    private MecanumDriveSubsystem driveSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private OuttakeSubsystem outtakeSubsystem;
    private TransferSubsystem transferSubsystem;
    private FlowersSubsystem flowersSubsystem;
    private VisionSubsystem visionSubsystem;

    private RobotStateManager stateManager;
    private GamepadEx driverGamepad;
    private GamepadEx operatorGamepad;

    @Override
    public void initialize() {
        // Dual Telemetry route
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        stateManager = new RobotStateManager(hardwareMap);
        stateManager.loadTunedPidGains(); // Load previously saved tuned PID values from robot memory

        driveSubsystem = new MecanumDriveSubsystem(hardwareMap);
        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        outtakeSubsystem = new OuttakeSubsystem(hardwareMap);
        transferSubsystem = new TransferSubsystem(hardwareMap);
        flowersSubsystem = new FlowersSubsystem(hardwareMap);
        visionSubsystem = new VisionSubsystem(hardwareMap);

        driverGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);

        // Default Drive Command
        driveSubsystem.setDefaultCommand(
                new DriveCommand(
                        driveSubsystem,
                        driverGamepad::getLeftX,
                        driverGamepad::getLeftY,
                        driverGamepad::getRightX
                )
        );

        // Gamepad 1 (Driver) Controls
        new GamepadButton(driverGamepad, GamepadKeys.Button.START)
                .whenPressed(new InstantCommand(driveSubsystem::resetHeading, driveSubsystem));

        new GamepadButton(driverGamepad, GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new InstantCommand(() -> driveSubsystem.setSpeedMode(MecanumDriveSubsystem.SpeedMode.TURBO)))
                .whenReleased(new InstantCommand(() -> driveSubsystem.setSpeedMode(MecanumDriveSubsystem.SpeedMode.NORMAL)));

        new GamepadButton(driverGamepad, GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(() -> driveSubsystem.setSpeedMode(MecanumDriveSubsystem.SpeedMode.SLOW)))
                .whenReleased(new InstantCommand(() -> driveSubsystem.setSpeedMode(MecanumDriveSubsystem.SpeedMode.NORMAL)));

        new GamepadButton(driverGamepad, GamepadKeys.Button.A)
                .whenPressed(new AlignToAprilTagCommand(driveSubsystem, visionSubsystem, -1, 12.0));

        // Gamepad 2 (Operator) Controls
        new GamepadButton(operatorGamepad, GamepadKeys.Button.DPAD_UP)
                .whenPressed(new OuttakeCommand(outtakeSubsystem, RobotConstants.OUTTAKE_HIGH_TARGET_HEIGHT));

        new GamepadButton(operatorGamepad, GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new OuttakeCommand(outtakeSubsystem, RobotConstants.OUTTAKE_LOW_TARGET_HEIGHT));

        new GamepadButton(operatorGamepad, GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new OuttakeCommand(outtakeSubsystem, RobotConstants.OUTTAKE_STOW_HEIGHT));

        new GamepadButton(operatorGamepad, GamepadKeys.Button.A)
                .toggleWhenPressed(new IntakeCommand(intakeSubsystem));

        new GamepadButton(operatorGamepad, GamepadKeys.Button.B)
                .whenPressed(new TransferCommand(transferSubsystem, 1.0));

        new GamepadButton(operatorGamepad, GamepadKeys.Button.X)
                .whenPressed(new ScoreSequenceCommand(outtakeSubsystem, transferSubsystem, RobotConstants.OUTTAKE_HIGH_TARGET_HEIGHT));

        new GamepadButton(operatorGamepad, GamepadKeys.Button.Y)
                .whenPressed(new FlowersScoreCommand(flowersSubsystem));

        // Live Gamepad PID Tuning Modifier (Hold BACK / SELECT + D-Pad to adjust OUTTAKE_KP live)
        new Trigger(() -> gamepad2.back && gamepad2.dpad_up)
                .whenActive(new InstantCommand(() -> RobotConstants.OUTTAKE_KP += 0.0005));
        new Trigger(() -> gamepad2.back && gamepad2.dpad_down)
                .whenActive(new InstantCommand(() -> RobotConstants.OUTTAKE_KP = Math.max(0.0, RobotConstants.OUTTAKE_KP - 0.0005)));
    }

    @Override
    public void run() {
        super.run();

        // Check if "SAVE_PID_VALUES_TO_ROBOT" button was clicked on Dashboard
        stateManager.checkAndHandleDashboardSave();

        // Comprehensive telemetry routed live to Driver Hub AND Dashboard
        telemetry.addData("=== DRIVETRAIN ===", "Heading: %.2f deg | FieldCentric: %b",
                Math.toDegrees(driveSubsystem.getHeadingRadians()), driveSubsystem.isFieldCentric());

        telemetry.addData("=== OUTTAKE SLIDE ===", "Current: %.2f in | Target: %.2f in | Kp: %.5f | AtTarget: %b",
                outtakeSubsystem.getCurrentHeightInches(), outtakeSubsystem.getTargetHeightInches(),
                RobotConstants.OUTTAKE_KP, outtakeSubsystem.isAtTarget());

        telemetry.addData("=== INTAKE / SENSOR ===", "HasBall: %b | Type: %s",
                intakeSubsystem.hasBall(), intakeSubsystem.detectBallType());

        BallVisionPipeline.BallTarget ballTarget = visionSubsystem.getTargetBall();
        if (ballTarget.detected) {
            telemetry.addData("=== VISION (CAMERA) ===", "Detected: %s | Radius: %.1f px | OffsetX: %.1f | OffsetY: %.1f",
                    ballTarget.type, ballTarget.radius, ballTarget.offsetX, ballTarget.offsetY);
        } else {
            telemetry.addData("=== VISION (CAMERA) ===", "No ball detected in camera frame");
        }

        telemetry.addData("=== FLOWERS ENDGAME ===", "Ticks: %.1f | Target: %.1f",
                flowersSubsystem.getCurrentTicks(), RobotConstants.FLOWERS_SCORE_POSITION);

        telemetry.update();
    }
}
