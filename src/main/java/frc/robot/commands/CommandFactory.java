package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.index.IndexCommands;
import frc.robot.subsystems.intake.IntakeCommands;
import frc.robot.subsystems.shooter.ShooterCommands;
import frc.robot.subsystems.shooter.Shooter.Positions;

public class CommandFactory {
    public static Command intakeCommand(){
        return new SequentialCommandGroup(
            IntakeCommands.intake(),
            IndexCommands.retract()
        );
    }

    public static Command shootCenterCommand(){
        return new SequentialCommandGroup(
            ShooterCommands.setAngle(Positions.SUB_LEFT),
            ShooterCommands.spinUp(),
            new WaitCommand(Autos.kSpinUpTime),
            ShooterCommands.shoot()
        );
    }
    
    public static Command shootSideCommand(){
        return new SequentialCommandGroup(
            ShooterCommands.setAngle(Positions.AUTO_SIDES),
            ShooterCommands.spinUp(),
            new WaitCommand(Autos.kSpinUpTime),
            ShooterCommands.shoot()
        );
    }
}