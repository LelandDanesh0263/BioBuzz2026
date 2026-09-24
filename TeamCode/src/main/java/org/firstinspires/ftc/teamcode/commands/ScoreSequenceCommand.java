package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;

public class ScoreSequenceCommand extends SequentialCommandGroup {

    public ScoreSequenceCommand(OuttakeSubsystem outtakeSubsystem, TransferSubsystem transferSubsystem, double targetHeightInches) {
        addCommands(
                // Step 1: Move outtake to target height
                new OuttakeCommand(outtakeSubsystem, targetHeightInches),

                // Step 2: Transfer ball into outtake claw/bucket
                new TransferCommand(transferSubsystem, 0.8),

                // Step 3: Mechanical settling pause
                new WaitCommand(150),

                // Step 4: Open outtake claw to drop Nectar/Pollen
                new InstantCommand(outtakeSubsystem::openClaw, outtakeSubsystem),

                // Step 5: Wait for deposit
                new WaitCommand(250),

                // Step 6: Close outtake claw & retract outtake slide to STOW
                new InstantCommand(outtakeSubsystem::closeClaw, outtakeSubsystem),
                new OuttakeCommand(outtakeSubsystem, RobotConstants.OUTTAKE_STOW_HEIGHT)
        );
    }
}
