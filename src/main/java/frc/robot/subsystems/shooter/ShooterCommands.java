package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.commands.Autos;
import frc.robot.subsystems.index.Index;

public class ShooterCommands {
    private static Index index = Index.getInstance();
    private static Shooter shooter = Shooter.getInstance();

    public static Command shoot(){
        return new StartEndCommand(
            () -> index.startTransfer(), 
            () -> {index.stopTransfer(); shooter.setShooterSpeed(0);}, 
            index).withTimeout(Autos.kShootTime);
    }

    public static Command spinUp(){
        return new InstantCommand(
            () -> shooter.setShooterSpeed(1),
            shooter
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