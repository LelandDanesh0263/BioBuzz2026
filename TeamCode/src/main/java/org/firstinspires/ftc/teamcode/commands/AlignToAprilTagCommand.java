package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.control.PIDController;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.vision.AprilTagPipeline;

public class AlignToAprilTagCommand extends CommandBase {

    private final MecanumDriveSubsystem driveSubsystem;
    private final VisionSubsystem visionSubsystem;
    private final int targetId;
    private final double targetDistanceInches;

    private final PIDController turnPID = new PIDController(0.03, 0.0, 0.002);
    private final PIDController forwardPID = new PIDController(0.04, 0.0, 0.003);

    private final ElapsedTime timeoutTimer = new ElapsedTime();

    public AlignToAprilTagCommand(MecanumDriveSubsystem driveSubsystem,
                                  VisionSubsystem visionSubsystem,
                                  int targetId,
                                  double targetDistanceInches) {
        this.driveSubsystem = driveSubsystem;
        this.visionSubsystem = visionSubsystem;
        this.targetId = targetId;
        this.targetDistanceInches = targetDistanceInches;
        addRequirements(driveSubsystem, visionSubsystem);
    }

    @Override
    public void initialize() {
        timeoutTimer.reset();
        turnPID.reset();
        forwardPID.reset();
    }

    @Override
    public void execute() {
        AprilTagPipeline.AprilTagTarget target = visionSubsystem.getTargetAprilTag(targetId);

        if (target.detected) {
            double turnPower = turnPID.calculate(0, target.bearingDegrees);
            double forwardPower = forwardPID.calculate(targetDistanceInches, target.rangeInches);

            double clampedTurn = Math.max(-0.4, Math.min(0.4, turnPower));
            double clampedForward = Math.max(-0.5, Math.min(0.5, forwardPower));

            driveSubsystem.drive(0, clampedForward, clampedTurn);
        } else {
            driveSubsystem.stop();
        }
    }

    @Override
    public boolean isFinished() {
        AprilTagPipeline.AprilTagTarget target = visionSubsystem.getTargetAprilTag(targetId);
        if (timeoutTimer.seconds() > 3.5) return true;

        if (target.detected) {
            boolean angleAligned = Math.abs(target.bearingDegrees) < 2.0;
            boolean distanceAligned = Math.abs(target.rangeInches - targetDistanceInches) < 1.0;
            return angleAligned && distanceAligned;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stop();
    }
}
