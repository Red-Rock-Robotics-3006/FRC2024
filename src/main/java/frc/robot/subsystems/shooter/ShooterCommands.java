package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.commands.Autos;
import frc.robot.subsystems.index.Index;
import frc.robot.subsystems.index.TOFSensor;

public class ShooterCommands {
    private static Index index = Index.getInstance();
    private static Shooter shooter = Shooter.getInstance();
    private static TOFSensor sensor = TOFSensor.getInstance();

    public static Command shoot(){
        // return new StartEndCommand(
        //     () -> index.startTransfer(), 
        //     () -> {index.stopTransfer(); shooter.setShooterSpeed(0);}, 
        //     index).withTimeout(Autos.kShootTime);
        return new FunctionalCommand(
            () -> index.startTransfer(), 
            () -> {}, 
            (interrupted) -> {index.stopTransfer(); shooter.setShooterSpeed(0);}, 
            () -> sensor.hasNote(), 
            index, shooter, sensor);
    }

    public static Command spinUp(){
        return new InstantCommand(
            () -> shooter.setShooterSpeed(0.5),
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

    public static Command stop(){
        return new InstantCommand(
            () -> shooter.setShooterSpeed(0),
            shooter
        );
    }

    public static Command setAngle(Shooter.Positions e) {
        return new InstantCommand(
            () -> {shooter.presetShoot(e); System.out.println("Shooter angle has been set to " + e);},//TODO remove print for testing
            shooter
        );
    }
}