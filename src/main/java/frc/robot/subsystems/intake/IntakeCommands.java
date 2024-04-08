package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.index.Index;
import frc.robot.subsystems.index.TOFSensor;
import frc.robot.subsystems.shooter.Shooter;

public class IntakeCommands {
    private static Intake intake = Intake.getInstance();
    private static IntakeVision intakeVision = IntakeVision.getInstance();
    private static Shooter shooter = Shooter.getInstance();
    private static Index index = Index.getInstance();
    private static TOFSensor sensor = TOFSensor.getInstance();
    
    
    public static Command start() {
        return new InstantCommand(
            () -> intake.startIntake(),
            intake
        );
    }

    public static Command stop() {
        return new InstantCommand(
            () -> intake.stopIntake(),
            intake
        );
    }
    
    public static Command reverse() {
        return new InstantCommand(
            () -> intake.reverseIntake(),
            intake
        );
    }

    public static Command intake() {
        return new FunctionalCommand(
            () -> {intake.startIntake(); index.startTransfer(); shooter.presetShoot(Shooter.Positions.INTAKE);}, 
            () -> {},
            (interrupted) -> {intake.stopIntake(); index.stopTransfer();}, 
            () -> sensor.hasNote(),
            intake, index, sensor, shooter
        );
    }

    public static Command toggleHoming() {
        return new InstantCommand(
            () -> intakeVision.toggleHoming(),
            intakeVision
        );
    }

    public static Command burnFlash() {
        return new SequentialCommandGroup(
            new WaitCommand(0.15),
            new InstantCommand(
                () -> intake.burnFlash(),
                intake
            )
        );
    }
}