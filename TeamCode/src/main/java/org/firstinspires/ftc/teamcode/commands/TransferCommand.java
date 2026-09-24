package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;

public class TransferCommand extends CommandBase {

    private final TransferSubsystem transferSubsystem;
    private final boolean reverse;
    private final double durationSeconds;
    private final ElapsedTime timer = new ElapsedTime();

    public TransferCommand(TransferSubsystem transferSubsystem, boolean reverse, double durationSeconds) {
        this.transferSubsystem = transferSubsystem;
        this.reverse = reverse;
        this.durationSeconds = durationSeconds;
        addRequirements(transferSubsystem);
    }

    public TransferCommand(TransferSubsystem transferSubsystem, double durationSeconds) {
        this(transferSubsystem, false, durationSeconds);
    }

    @Override
    public void initialize() {
        timer.reset();
        if (reverse) {
            transferSubsystem.transferReverse();
        } else {
            transferSubsystem.transferForward();
        }
    }

    @Override
    public boolean isFinished() {
        return timer.seconds() >= durationSeconds;
    }

    @Override
    public void end(boolean interrupted) {
        transferSubsystem.stopTransfer();
    }
}
