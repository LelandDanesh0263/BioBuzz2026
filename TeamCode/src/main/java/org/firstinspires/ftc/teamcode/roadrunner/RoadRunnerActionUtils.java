package org.firstinspires.ftc.teamcode.roadrunner;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;

public class RoadRunnerActionUtils {

    /**
     * Converts a RoadRunner Action into an FTCLib Command that can be scheduled or added to CommandGroups.
     */
    public static Command actionToCommand(Action action) {
        return new CommandBase() {
            private boolean finished = false;

            @Override
            public void execute() {
                TelemetryPacket packet = new TelemetryPacket();
                finished = !action.run(packet);
            }

            @Override
            public boolean isFinished() {
                return finished;
            }
        };
    }

    /**
     * Converts an FTCLib Command into a RoadRunner Action that can be chained inside RoadRunner trajectory sequences.
     */
    public static Action commandToAction(Command command) {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(TelemetryPacket packet) {
                if (!initialized) {
                    command.initialize();
                    initialized = true;
                }
                command.execute();
                if (command.isFinished()) {
                    command.end(false);
                    return false;
                }
                return true;
            }
        };
    }
}
