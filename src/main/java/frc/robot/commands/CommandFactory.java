package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.shooter.ShooterCommands;
import frc.robot.subsystems.shooter.Shooter.Positions;

public class CommandFactory {
    public static Command shootCenterCommand(){
        return new SequentialCommandGroup(
            ShooterCommands.setAngle(Positions.SUB_LEFT),
            ShooterCommands.spinUp(),
            new WaitCommand(Autos.kSpinUpTime),
            ShooterCommands.shootAuto()
        );
    }
    
    public static Command shootSideCommand(){
        return new SequentialCommandGroup(
            ShooterCommands.setAngle(Positions.SUB_LEFT),
            ShooterCommands.spinUp(),
            new WaitCommand(1),
            ShooterCommands.shootAuto()
        );
    }
}