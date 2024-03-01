package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Shooter.Positions;
import frc.robot.subsystems.swerve.generated.TunerConstants;

public class Autos {
    public static Command oneNoteGrabAuto(){
        return new SequentialCommandGroup(
            new InstantCommand(() -> TunerConstants.DriveTrain.seedFieldRelative(new Pose2d(1.34, 5.53, new Rotation2d())), TunerConstants.DriveTrain),
            new InstantCommand(() -> Shooter.getInstance().presetShoot(Shooter.Positions.SUB_LEFT), Shooter.getInstance()),
            new InstantCommand(() -> Shooter.getInstance().setShooterSpeed(1), Shooter.getInstance()),
            new WaitCommand(2.5),
            new InstantCommand(
                () -> Index.getInstance().startTransfer(),
                Index.getInstance()
            ),
            new WaitCommand(2),
            new InstantCommand(
                () -> {
                    Index.getInstance().stopTransfer();
                    Shooter.getInstance().setShooterSpeed(0);
                },
                Index.getInstance(),
                Shooter.getInstance()
            ),
            new InstantCommand(
                () -> {
                    Intake.getInstance().setHoming(true);
                }
            ),
            TunerConstants.DriveTrain.getAuto("MidNote")
        );
    }

    public static Command twoNoteAuto(){
        return new SequentialCommandGroup(
            new InstantCommand(() -> TunerConstants.DriveTrain.seedFieldRelative(new Pose2d(1.34, 5.53, new Rotation2d())), TunerConstants.DriveTrain),
            new InstantCommand(() -> Shooter.getInstance().presetShoot(Shooter.Positions.SUB_LEFT), Shooter.getInstance()),
            new InstantCommand(() -> Shooter.getInstance().setShooterSpeed(1), Shooter.getInstance()),
            new WaitCommand(2.5),
            new InstantCommand(
                () -> Index.getInstance().startTransfer(),
                Index.getInstance()
            ),
            new WaitCommand(2),
            new InstantCommand(
                () -> {
                    Index.getInstance().stopTransfer();
                    Shooter.getInstance().setShooterSpeed(0);
                },
                Index.getInstance(),
                Shooter.getInstance()
            ),
            new InstantCommand(
                () -> {
                    Intake.getInstance().setHoming(true);
                }
            ),
            TunerConstants.DriveTrain.getAuto("MidNote"),
            new WaitCommand(0.5),
            new InstantCommand(() -> Shooter.getInstance().setShooterSpeed(1), Shooter.getInstance()),
            TunerConstants.DriveTrain.goToPose(new Pose2d(1.3, 5.53, new Rotation2d())),
            new WaitCommand(0.5),
            new StartEndCommand(
                () -> Index.getInstance().startTransfer(), 
                () -> Index.getInstance().stopTransfer(), 
                Index.getInstance()
            ).withTimeout(2)
        );
    }
    public static Command oneNoteSourceSide(){
        return new SequentialCommandGroup(
            new InstantCommand(() -> TunerConstants.DriveTrain.seedFieldRelative(new Pose2d(1.34, 5.53, new Rotation2d())), TunerConstants.DriveTrain),
            new InstantCommand(() -> Shooter.getInstance().presetShoot(Shooter.Positions.SUB_LEFT), Shooter.getInstance()),
            new InstantCommand(() -> Shooter.getInstance().setShooterSpeed(1), Shooter.getInstance()),
            new WaitCommand(2.5),
            new InstantCommand(
                () -> Index.getInstance().startTransfer(),
                Index.getInstance()
            ),
            new WaitCommand(2),
            new InstantCommand(
                () -> {
                    Index.getInstance().stopTransfer();
                    Shooter.getInstance().setShooterSpeed(0);
                },
                Index.getInstance(),
                Shooter.getInstance()
            ),
            TunerConstants.DriveTrain.getAuto("MidNote")
        );
    }

    public static Command twoNotePaths(){
        return new SequentialCommandGroup(
            TunerConstants.DriveTrain.getAuto("MidNote"),
            TunerConstants.DriveTrain.goToPose(new Pose2d(1.3,5.53, new Rotation2d()))
        );
    }

    public static Command blueSourceToCenter(){
        return new SequentialCommandGroup(
            new InstantCommand(() -> TunerConstants.DriveTrain.seedFieldRelative(new Pose2d(1.34, 5.53, new Rotation2d())), TunerConstants.DriveTrain),
            new InstantCommand(() -> Shooter.getInstance().presetShoot(Shooter.Positions.SUB_LEFT), Shooter.getInstance()),
            new InstantCommand(() -> Shooter.getInstance().setShooterSpeed(1), Shooter.getInstance()),
            new WaitCommand(2.5),
            new InstantCommand(
                () -> Index.getInstance().startTransfer(),
                Index.getInstance()
            ),
            new WaitCommand(2),
            new InstantCommand(
                () -> {
                    Index.getInstance().stopTransfer();
                    Shooter.getInstance().setShooterSpeed(0);
                },
                Index.getInstance(),
                Shooter.getInstance()
            ),
            // new InstantCommand(
            //     () -> {
            //         Intake.getInstance().setHoming(true);
            //     }
            // ),
            TunerConstants.DriveTrain.getAuto("BlueSourceToCenterAuto")
        );
    }
    public static Command redSourceToCenter(){
        return new SequentialCommandGroup(
            new InstantCommand(() -> TunerConstants.DriveTrain.seedFieldRelative(new Pose2d(1.34, 5.53, new Rotation2d())), TunerConstants.DriveTrain),
            new InstantCommand(() -> Shooter.getInstance().presetShoot(Shooter.Positions.SUB_LEFT), Shooter.getInstance()),
            new InstantCommand(() -> Shooter.getInstance().setShooterSpeed(1), Shooter.getInstance()),
            new WaitCommand(2.5),
            new InstantCommand(
                () -> Index.getInstance().startTransfer(),
                Index.getInstance()
            ),
            new WaitCommand(2),
            new InstantCommand(
                () -> {
                    Index.getInstance().stopTransfer();
                    Shooter.getInstance().setShooterSpeed(0);
                },
                Index.getInstance(),
                Shooter.getInstance()
            ),
            // new InstantCommand(
            //     () -> {
            //         Intake.getInstance().setHoming(true);
            //     }
            // ),
            TunerConstants.DriveTrain.getAuto("RedSourceToCenterAuto")
        );
    }
    public static Command blueAmpNoPickup(){
        return new SequentialCommandGroup(
            new InstantCommand(() -> TunerConstants.DriveTrain.seedFieldRelative(new Pose2d(1.34, 5.53, new Rotation2d())), TunerConstants.DriveTrain),
            new InstantCommand(() -> Shooter.getInstance().presetShoot(Shooter.Positions.SUB_LEFT), Shooter.getInstance()),
            new InstantCommand(() -> Shooter.getInstance().setShooterSpeed(1), Shooter.getInstance()),
            new WaitCommand(2.5),
            new InstantCommand(
                () -> Index.getInstance().startTransfer(),
                Index.getInstance()
            ),
            new WaitCommand(2),
            new InstantCommand(
                () -> {
                    Index.getInstance().stopTransfer();
                    Shooter.getInstance().setShooterSpeed(0);
                },
                Index.getInstance(),
                Shooter.getInstance()
            ),
            // new InstantCommand(
            //     () -> {
            //         Intake.getInstance().setHoming(true);
            //     }
            // ),
            TunerConstants.DriveTrain.getAuto("BlueAmpNoPickupAuto")
        );
    }
}

