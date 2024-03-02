package frc.robot.commands;

import java.time.Instant;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    
    public static Command justShootSides(){
        return new SequentialCommandGroup(
            new InstantCommand(() -> TunerConstants.DriveTrain.seedFieldRelative(new Pose2d(1.34, 5.53, new Rotation2d())), TunerConstants.DriveTrain),
            new InstantCommand(() -> Shooter.getInstance().presetShoot(Shooter.Positions.AUTO_SIDES), Shooter.getInstance()),
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
            )
        );
    }
    public static Command twoNoteAuto(){   
        return new SequentialCommandGroup(
            new InstantCommand(() -> TunerConstants.DriveTrain.seedFieldRelative(new Pose2d(1.34, 5.53, new Rotation2d())), TunerConstants.DriveTrain),
            new InstantCommand(() -> Shooter.getInstance().presetShoot(Shooter.Positions.SUB_LEFT), Shooter.getInstance()),
            new InstantCommand(() -> Shooter.getInstance().setShooterSpeed(1), Shooter.getInstance()),
            new WaitCommand(2),
            new InstantCommand(
                () -> Index.getInstance().startTransfer(),
                Index.getInstance()
            ),
            new WaitCommand(1),
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
            new WaitCommand(0.3),
            new InstantCommand(() -> Shooter.getInstance().setShooterSpeed(1), Shooter.getInstance()),
            TunerConstants.DriveTrain.getAuto("MidNoteBackAuto"),
            new InstantCommand(() -> System.out.println("hi")),
            new WaitCommand(0.5),
            new StartEndCommand(
                () -> Index.getInstance().startTransfer(), 
                () -> Index.getInstance().stopTransfer(), 
                Index.getInstance()
            ).withTimeout(1),
            new InstantCommand(() -> Shooter.getInstance().setShooterSpeed(0), Shooter.getInstance())
        );
    }
    
    public static Command twoNoteAuto2(){
        return new SequentialCommandGroup(
            new InstantCommand(() -> TunerConstants.DriveTrain.seedFieldRelative(new Pose2d(1.34, 5.53, new Rotation2d())), TunerConstants.DriveTrain),
            new InstantCommand(() -> Shooter.getInstance().presetShoot(Shooter.Positions.SUB_LEFT), Shooter.getInstance()),
            new InstantCommand(() -> Shooter.getInstance().setShooterSpeed(1), Shooter.getInstance()),
            new WaitCommand(1.5),
            new InstantCommand(
                () -> Index.getInstance().startTransfer(),
                Index.getInstance()
            ),
            new WaitCommand(0.7),
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
            new WaitCommand(0.3),
            new InstantCommand(() -> Shooter.getInstance().setShooterSpeed(1), Shooter.getInstance()),
            TunerConstants.DriveTrain.getAuto("MidNoteForwardBackAuto"),
            new InstantCommand(() -> SmartDashboard.putBoolean("pancreatic cancer", true))
            // TunerConstants.DriveTrain.getAuto("MidNoteBackAuto"),
            // new InstantCommand(() -> System.out.println("hi")),
            // new WaitCommand(0.5),
            // new StartEndCommand(
            //     () -> Index.getInstance().startTransfer(), 
            //     () -> Index.getInstance().stopTransfer(), .
            //     Index.getInstance()
            // ).withTimeout(0.7),
            // new InstantCommand(() -> Shooter.getInstance().setShooterSpeed(0), Shooter.getInstance())
        );
    }
    public static Command oneNoteSourceSide(){
        return new SequentialCommandGroup(
            new InstantCommand(() -> TunerConstants.DriveTrain.seedFieldRelative(new Pose2d(1.34, 5.53, new Rotation2d())), TunerConstants.DriveTrain),
            new InstantCommand(() -> Shooter.getInstance().presetShoot(Shooter.Positions.AUTO_SIDES), Shooter.getInstance()),
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
            TunerConstants.DriveTrain.getAuto("SourceNoPickup")
        );
    }

    public static Command twoNotePaths(){
        return new SequentialCommandGroup(
            TunerConstants.DriveTrain.getAuto("MidNote"),
            TunerConstants.DriveTrain.getAuto("MidNoteBackAuto")
        );
    }

    public static Command blueSourceToCenter(){
        return new SequentialCommandGroup(
            new InstantCommand(() -> TunerConstants.DriveTrain.seedFieldRelative(new Pose2d(1.34, 5.53, new Rotation2d())), TunerConstants.DriveTrain),
            new InstantCommand(() -> Shooter.getInstance().presetShoot(Shooter.Positions.AUTO_SIDES), Shooter.getInstance()),
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
            new InstantCommand(() -> Shooter.getInstance().presetShoot(Shooter.Positions.AUTO_SIDES), Shooter.getInstance()),
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
            new InstantCommand(() -> Shooter.getInstance().presetShoot(Shooter.Positions.AUTO_SIDES), Shooter.getInstance()),
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
    
    public static Command redAmpNoPickup(){
        return new SequentialCommandGroup(
            new InstantCommand(() -> TunerConstants.DriveTrain.seedFieldRelative(new Pose2d(1.34, 5.53, new Rotation2d())), TunerConstants.DriveTrain),
            new InstantCommand(() -> Shooter.getInstance().presetShoot(Shooter.Positions.AUTO_SIDES), Shooter.getInstance()),
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
            TunerConstants.DriveTrain.getAuto("RedAmpNoPickupAuto")
        );
    }

    public static Command test(){
        return new SequentialCommandGroup(
            TunerConstants.DriveTrain.getAuto("MidNote"),
            new InstantCommand(() -> SmartDashboard.putBoolean("i got pancreatic cancer", true)),
            new WaitCommand(1),
            TunerConstants.DriveTrain.getAuto("MidNoteBackAuto"),
            new InstantCommand(() -> SmartDashboard.putBoolean("i got pancreatic cancer", false))
        );
    }

}

