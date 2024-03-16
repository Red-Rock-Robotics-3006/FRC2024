package frc.robot.subsystems.index;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;

public class IndexCommands {
    private static Index index = Index.getInstance();
    
    public static Command start() {
        return new InstantCommand(
            () -> index.startTransfer(),
            index
        );
    }

    public static Command shootStart() {
        return new InstantCommand(
            () -> index.shootTransfer(),
            index
        );
    }

    public static Command stop() {
        return new InstantCommand(
            () -> index.stopTransfer(),
            index
        );
    }

    public static Command reverse() {
        return new InstantCommand(
            () -> index.reverseTransfer(),
            index
        );
    }

    public static Command retract() {
        return new StartEndCommand(
            () -> index.reverseTransfer(),
            () -> index.stopTransfer(),
            index
        ).withTimeout(SmartDashboard.getNumber("index reverse time", Index.kReverseTime));
    }
}
