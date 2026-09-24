package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;

public class LiftToHeightCommand extends OuttakeCommand {

    public LiftToHeightCommand(OuttakeSubsystem outtakeSubsystem, double targetHeightInches) {
        super(outtakeSubsystem, targetHeightInches);
    }

    public LiftToHeightCommand(OuttakeSubsystem outtakeSubsystem, double targetHeightInches, double timeoutSeconds) {
        super(outtakeSubsystem, targetHeightInches, timeoutSeconds);
    }
}
