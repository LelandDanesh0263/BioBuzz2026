package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;

public class OuttakeCommand extends CommandBase {

    private final OuttakeSubsystem outtakeSubsystem;
    private final double targetHeightInches;
    private final double timeoutSeconds;
    private final ElapsedTime timer = new ElapsedTime();

    public OuttakeCommand(OuttakeSubsystem outtakeSubsystem, double targetHeightInches, double timeoutSeconds) {
        this.outtakeSubsystem = outtakeSubsystem;
        this.targetHeightInches = targetHeightInches;
        this.timeoutSeconds = timeoutSeconds;
        addRequirements(outtakeSubsystem);
    }

    public OuttakeCommand(OuttakeSubsystem outtakeSubsystem, double targetHeightInches) {
        this(outtakeSubsystem, targetHeightInches, 3.0);
    }

    @Override
    public void initialize() {
        timer.reset();
        outtakeSubsystem.setTargetHeightInches(targetHeightInches);
    }

    @Override
    public boolean isFinished() {
        return outtakeSubsystem.isAtTarget() || timer.seconds() >= timeoutSeconds;
    }
}
