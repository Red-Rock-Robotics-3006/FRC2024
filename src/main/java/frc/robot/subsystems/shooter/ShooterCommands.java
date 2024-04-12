package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.index.Index;
import frc.robot.subsystems.index.TOFSensor;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.led.State;

public class ShooterCommands {
    private static Index index = Index.getInstance();
    private static Shooter shooter = Shooter.getInstance();
    private static TOFSensor sensor = TOFSensor.getInstance();
    private static LED led = LED.getInstance();

    public static Command shoot() {
        return new FunctionalCommand(
            () -> index.shootTransfer(), 
            () -> {}, 
            (interrupted) -> {shooter.stow(); index.stopTransfer(); led.setState(State.RESTING); led.setIsAmping(false);},
            () -> !sensor.hasNote(), 
            index, shooter, sensor
        );
    }

    public static Command shootAuto() {
        return new FunctionalCommand(
            () -> index.shootTransfer(), 
            () -> {}, 
            (interrupted) -> {index.stopTransfer(); shooter.setHoming(false); led.setState(State.RESTING);},
            () -> !sensor.hasNote(), 
            index, shooter, sensor
        );
    }

    public static Command fullCourtLob() {
        return new FunctionalCommand(
            () -> {shooter.runFullLobShot(); shooter.runFullLobAngle(); shooter.setRunningFullLob(true);}, 
            () -> {}, 
            (interrupted) -> {shooter.setRunningFullLob(false);},
            () -> !sensor.hasNote(), 
            shooter, sensor
        );
    }

    public static Command spinUp() {
        return new InstantCommand(
            () -> shooter.setShooterSpeed(0.5),
            shooter
        );
    }

    public static Command trollSpinUp() {
        return new InstantCommand(
            () -> shooter.setShooterSpeed(0.1),
            shooter
        );
    }

    public static Command ampSpinUp() {
        return new SequentialCommandGroup(
            new InstantCommand(
                () -> shooter.runAmpAngle(),
                shooter
            ),
            new InstantCommand(
                () -> shooter.runAmpShot(),
                shooter
            )
        );
    }

    public static Command setShooterSpeed(double speed) {
        return new InstantCommand(
            () -> shooter.setShooterSpeed(speed),
            shooter
        );
    }

    public static Command stop() {
        return new InstantCommand(
            () -> shooter.setShooterSpeed(0),
            shooter
        );
    }

    public static Command stow() {
        return new InstantCommand(
            () -> shooter.stow(),
            shooter
        );
    }

    public static Command setAngle(Shooter.Positions e) {
        return new InstantCommand(
            () -> shooter.presetShoot(e),
            shooter
        );
    }

    public static Command setTarget(double angle) {
        return new InstantCommand(
            () -> shooter.setTarget(angle),
            shooter
        );
    }

    public static Command increaseTarget() {
        return new InstantCommand(
            () -> shooter.incrementTarget(),
            shooter
        );
    }

    public static Command decreaseTarget() {
        return new InstantCommand(
            () -> shooter.decrementTarget(),
            shooter
        );
    }

    public static Command setHoming(boolean e) {
        return new InstantCommand(
            () -> shooter.setHoming(e),
            shooter
        );
    }

    public static Command increaseDistanceFeed() {
        return new InstantCommand(
            () -> shooter.increaseDistanceFeed(),
            shooter
        );
    }

    public static Command decreaseDistanceFeed() {
        return new InstantCommand(
            () -> shooter.decreaseDistanceFeed(),
            shooter
        );
    }

    public static Command burnFlash() {
        return new SequentialCommandGroup(
            new WaitCommand(0.15),
            new InstantCommand(
                () -> shooter.burnFlash(),
                shooter
            )
        );
    }
}