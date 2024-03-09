package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Shooter.Positions;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.generated.TunerConstants;

public class CommandFactory {
    private static Intake intake = Intake.getInstance();
    private static Shooter shooter = Shooter.getInstance();
    private static Index index = Index.getInstance();
    private static CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;

    public static Command intakeCommand(){
        return new SequentialCommandGroup(
            new FunctionalCommand(
                () -> intake.startIntake(), 
                () -> {},
                (interrupted) -> intake.stopIntake(), 
                () -> index.hasNote(), 
                intake, index),
            new StartEndCommand(
                () -> index.reverseFastTransfer(), 
                () -> index.stopTransfer(),
                index 
            ).withTimeout(SmartDashboard.getNumber("index reverse time", Index.kReverseTime))
        );
    }

    public static Command shootCenterCommand(){
        return new SequentialCommandGroup(
            new InstantCommand(() -> {
                shooter.presetShoot(Positions.SUB_LEFT);
                shooter.setShooterSpeed(1);
            }, shooter),
            new WaitCommand(Autos.kSpinUpTime),
            new StartEndCommand(
                () -> index.startTransfer(), 
                () -> index.stopTransfer(), 
                index).withTimeout(Autos.kShootTime)
        );
    }
    
    public static Command shootSideCommand(){
        return new SequentialCommandGroup(
            new InstantCommand(() -> {
                shooter.presetShoot(Positions.AUTO_SIDES);
                shooter.setShooterSpeed(1);
            }, shooter),
            new WaitCommand(Autos.kSpinUpTime),
            new StartEndCommand(
                () -> index.startTransfer(), 
                () -> {index.stopTransfer(); shooter.setShooterSpeed(0);}, 
                index).withTimeout(Autos.kShootTime)
        );
    }

    public static Command shootCommand(){
        return             new StartEndCommand(
                () -> index.startTransfer(), 
                () -> {index.stopTransfer(); shooter.setShooterSpeed(0);}, 
                index).withTimeout(Autos.kShootTime);
    }

    public static Command spinUpCommand(){
        return new InstantCommand(() -> shooter.setShooterSpeed(1), shooter);
    }


}
