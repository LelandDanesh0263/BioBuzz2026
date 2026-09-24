package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

import java.util.function.DoubleSupplier;

public class DriveCommand extends CommandBase {

    private final MecanumDriveSubsystem driveSubsystem;
    private final DoubleSupplier strafeSupplier;
    private final DoubleSupplier forwardSupplier;
    private final DoubleSupplier turnSupplier;

    public DriveCommand(MecanumDriveSubsystem driveSubsystem,
                        DoubleSupplier strafeSupplier,
                        DoubleSupplier forwardSupplier,
                        DoubleSupplier turnSupplier) {
        this.driveSubsystem = driveSubsystem;
        this.strafeSupplier = strafeSupplier;
        this.forwardSupplier = forwardSupplier;
        this.turnSupplier = turnSupplier;
        addRequirements(driveSubsystem);
    }

    private double applyDeadband(double value) {
        if (Math.abs(value) < 0.05) return 0.0;
        return Math.copySign(Math.pow(value, 2), value);
    }

    @Override
    public void execute() {
        double strafe = applyDeadband(strafeSupplier.getAsDouble());
        double forward = applyDeadband(forwardSupplier.getAsDouble());
        double turn = applyDeadband(turnSupplier.getAsDouble());

        driveSubsystem.drive(strafe, forward, turn);
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stop();
    }
}
