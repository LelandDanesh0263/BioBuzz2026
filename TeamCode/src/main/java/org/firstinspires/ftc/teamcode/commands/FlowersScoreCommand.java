package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import org.firstinspires.ftc.teamcode.subsystems.FlowersSubsystem;

public class FlowersScoreCommand extends SequentialCommandGroup {

    public FlowersScoreCommand(FlowersSubsystem flowersSubsystem) {
        addCommands(
                new InstantCommand(flowersSubsystem::deployForFlowersScore, flowersSubsystem),
                new WaitCommand(600),
                new InstantCommand(flowersSubsystem::extendPusher, flowersSubsystem),
                new WaitCommand(400),
                new InstantCommand(flowersSubsystem::retractPusher, flowersSubsystem),
                new WaitCommand(200),
                new InstantCommand(flowersSubsystem::stow, flowersSubsystem)
        );
    }
}
