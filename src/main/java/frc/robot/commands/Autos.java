package frc.robot.commands;

import java.time.Instant;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerTrajectory;

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
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.generated.TunerConstants;

public class Autos {

    public static Intake intake = Intake.getInstance();
    public static Index index = Index.getInstance();
    public static Shooter shooter = Shooter.getInstance();
    public static CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;

    public static double kSpinUpTime = 1;
    public static double kShootTime = 0.3;

    public static Command oneNoteGrabAuto(){
        return new SequentialCommandGroup(
            new InstantCommand(() -> TunerConstants.DriveTrain.seedFieldRelative(new Pose2d(1.34, 5.53, new Rotation2d())), TunerConstants.DriveTrain),
            new InstantCommand(() -> shooter.presetShoot(Shooter.Positions.SUB_LEFT), shooter),
            new InstantCommand(() -> shooter.setShooterSpeed(1), shooter),
            new WaitCommand(2.5),
            new InstantCommand(
                () -> index.startTransfer(),
                index
            ),
            new WaitCommand(2),
            new InstantCommand(
                () -> {
                    index.stopTransfer();
                    shooter.setShooterSpeed(0);
                },
                index,
                shooter
            ),
            new InstantCommand(
                () -> {
                    intake.setHoming(true);
                }
            ),
            TunerConstants.DriveTrain.getAuto("MidNote")
        );
    }
    
    public static Command justShootSides(){
        return new SequentialCommandGroup(
            new InstantCommand(() -> TunerConstants.DriveTrain.seedFieldRelative(new Pose2d(1.34, 5.53, new Rotation2d())), TunerConstants.DriveTrain),
            new InstantCommand(() -> shooter.presetShoot(Shooter.Positions.AUTO_SIDES), shooter),
            new InstantCommand(() -> shooter.setShooterSpeed(1), shooter),
            new WaitCommand(2.5),
            new InstantCommand(
                () -> index.startTransfer(),
                index
            ),
            new WaitCommand(2),
            new InstantCommand(
                () -> {
                    index.stopTransfer();
                    shooter.setShooterSpeed(0);
                },
                index,
                shooter
            )
        );
    }
    public static Command twoNoteAuto(){   
        return new SequentialCommandGroup(
            new InstantCommand(() -> TunerConstants.DriveTrain.seedFieldRelative(new Pose2d(1.34, 5.53, new Rotation2d())), TunerConstants.DriveTrain),
            new InstantCommand(() -> shooter.presetShoot(Shooter.Positions.SUB_LEFT), shooter),
            new InstantCommand(() -> shooter.setShooterSpeed(1), shooter),
            new WaitCommand(2),
            new InstantCommand(
                () -> index.startTransfer(),
                index
            ),
            new WaitCommand(1),
            new InstantCommand(
                () -> {
                    index.stopTransfer();
                    shooter.setShooterSpeed(0);
                },
                index,
                shooter
            ),
            new InstantCommand(
                () -> {
                    intake.setHoming(true);
                }
            ),
            TunerConstants.DriveTrain.getAuto("MidNote"),
            new WaitCommand(0.3),
            new InstantCommand(() -> shooter.setShooterSpeed(1), shooter),
            TunerConstants.DriveTrain.getAuto("MidNoteBackAuto"),
            new InstantCommand(() -> System.out.println("hi")),
            new WaitCommand(0.5),
            new StartEndCommand(
                () -> index.startTransfer(), 
                () -> index.stopTransfer(), 
                index
            ).withTimeout(1),
            new InstantCommand(() -> shooter.setShooterSpeed(0), shooter)
        );
    }
    
    public static Command twoNoteAuto2(){
        return new SequentialCommandGroup(
            new InstantCommand(() -> TunerConstants.DriveTrain.seedFieldRelative(new Pose2d(1.34, 5.53, new Rotation2d())), TunerConstants.DriveTrain),
            new InstantCommand(() -> shooter.presetShoot(Shooter.Positions.SUB_LEFT), shooter),
            new InstantCommand(() -> shooter.setShooterSpeed(1), shooter),
            new WaitCommand(1.5),
            new InstantCommand(
                () -> index.startTransfer(),
                index
            ),
            new WaitCommand(0.7),
            new InstantCommand(
                () -> {
                    index.stopTransfer();
                    shooter.setShooterSpeed(0);
                },
                index,
                shooter
            ),
            new InstantCommand(
                () -> {
                    intake.setHoming(true);
                }
            ),
            TunerConstants.DriveTrain.getAuto("MidNote"),
            new WaitCommand(0.3),
            new InstantCommand(() -> shooter.setShooterSpeed(1), shooter),
            TunerConstants.DriveTrain.getAuto("MidNoteForwardBackAuto"),
            new InstantCommand(() -> SmartDashboard.putBoolean("pancreatic cancer", true))
            // TunerConstants.DriveTrain.getAuto("MidNoteBackAuto"),
            // new InstantCommand(() -> System.out.println("hi")),
            // new WaitCommand(0.5),
            // new StartEndCommand(
            //     () -> index.startTransfer(), 
            //     () -> index.stopTransfer(), .
            //     index
            // ).withTimeout(0.7),
            // new InstantCommand(() -> shooter.setShooterSpeed(0), shooter)
        );
    }
    public static Command oneNoteSourceSide(){
        return new SequentialCommandGroup(
            new InstantCommand(() -> TunerConstants.DriveTrain.seedFieldRelative(new Pose2d(1.34, 5.53, new Rotation2d())), TunerConstants.DriveTrain),
            new InstantCommand(() -> shooter.presetShoot(Shooter.Positions.AUTO_SIDES), shooter),
            new InstantCommand(() -> shooter.setShooterSpeed(1), shooter),
            new WaitCommand(2.5),
            new InstantCommand(
                () -> index.startTransfer(),
                index
            ),
            new WaitCommand(2),
            new InstantCommand(
                () -> {
                    index.stopTransfer();
                    shooter.setShooterSpeed(0);
                },
                index,
                shooter
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
            new InstantCommand(() -> shooter.presetShoot(Shooter.Positions.AUTO_SIDES), shooter),
            new InstantCommand(() -> shooter.setShooterSpeed(1), shooter),
            new WaitCommand(2.5),
            new InstantCommand(
                () -> index.startTransfer(),
                index
            ),
            new WaitCommand(2),
            new InstantCommand(
                () -> {
                    index.stopTransfer();
                    shooter.setShooterSpeed(0);
                },
                index,
                shooter
            ),
            // new InstantCommand(
            //     () -> {
            //         intake.setHoming(true);
            //     }
            // ),
            TunerConstants.DriveTrain.getAuto("BlueSourceToCenterAuto")
        );
    }
    public static Command redSourceToCenter(){
        return new SequentialCommandGroup(
            new InstantCommand(() -> TunerConstants.DriveTrain.seedFieldRelative(new Pose2d(1.34, 5.53, new Rotation2d())), TunerConstants.DriveTrain),
            new InstantCommand(() -> shooter.presetShoot(Shooter.Positions.AUTO_SIDES), shooter),
            new InstantCommand(() -> shooter.setShooterSpeed(1), shooter),
            new WaitCommand(2.5),
            new InstantCommand(
                () -> index.startTransfer(),
                index
            ),
            new WaitCommand(2),
            new InstantCommand(
                () -> {
                    index.stopTransfer();
                    shooter.setShooterSpeed(0);
                },
                index,
                shooter
            ),
            // new InstantCommand(
            //     () -> {
            //         intake.setHoming(true);
            //     }
            // ),
            TunerConstants.DriveTrain.getAuto("RedSourceToCenterAuto")
        );
    }
    public static Command blueAmpNoPickup(){
        return new SequentialCommandGroup(
            new InstantCommand(() -> TunerConstants.DriveTrain.seedFieldRelative(new Pose2d(1.34, 5.53, new Rotation2d())), TunerConstants.DriveTrain),
            new InstantCommand(() -> shooter.presetShoot(Shooter.Positions.AUTO_SIDES), shooter),
            new InstantCommand(() -> shooter.setShooterSpeed(1), shooter),
            new WaitCommand(2.5),
            new InstantCommand(
                () -> index.startTransfer(),
                index
            ),
            new WaitCommand(2),
            new InstantCommand(
                () -> {
                    index.stopTransfer();
                    shooter.setShooterSpeed(0);
                },
                index,
                shooter
            ),
            // new InstantCommand(
            //     () -> {
            //         intake.setHoming(true);
            //     }
            // ),
            TunerConstants.DriveTrain.getAuto("BlueAmpNoPickupAuto")
        );
    }   
    
    public static Command redAmpNoPickup(){
        return new SequentialCommandGroup(
            new InstantCommand(() -> TunerConstants.DriveTrain.seedFieldRelative(new Pose2d(1.34, 5.53, new Rotation2d())), TunerConstants.DriveTrain),
            new InstantCommand(() -> shooter.presetShoot(Shooter.Positions.AUTO_SIDES), shooter),
            new InstantCommand(() -> shooter.setShooterSpeed(1), shooter),
            new WaitCommand(2.5),
            new InstantCommand(
                () -> index.startTransfer(),
                index
            ),
            new WaitCommand(2),
            new InstantCommand(
                () -> {
                    index.stopTransfer();
                    shooter.setShooterSpeed(0);
                },
                index,
                shooter
            ),
            // new InstantCommand(
            //     () -> {
            //         intake.setHoming(true);
            //     }
            // ),
            TunerConstants.DriveTrain.getAuto("RedAmpNoPickupAuto")
        );
    }

    public static Command trollAuto() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> TunerConstants.DriveTrain.seedFieldRelative(new Pose2d(1.34, 5.53, new Rotation2d())), TunerConstants.DriveTrain),
            new InstantCommand(() -> shooter.presetShoot(Shooter.Positions.SUB_LEFT), shooter),
            new InstantCommand(() -> shooter.setShooterSpeed(1), shooter),
            new WaitCommand(kSpinUpTime),
            new InstantCommand(() -> index.startTransfer(), index),
            new WaitCommand(kShootTime),
            new InstantCommand(() -> intake.stopIntake()),
            new InstantCommand(() -> intake.startIntake(), intake),
            new InstantCommand(() -> shooter.setShooterSpeed(0.3), shooter),
            TunerConstants.DriveTrain.getAuto("TrollAuto"),
            new InstantCommand(() -> intake.stopIntake(), intake)
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

    public static Command m_4_1p_3w(){
        return new SequentialCommandGroup(
            drivetrain.getAuto("4NF_1"),
            drivetrain.getAuto("4NF_2"),
            drivetrain.getAuto("4NF_3"),
            drivetrain.getAuto("4NF_4"),
            drivetrain.getAuto("4NF_5"),
            drivetrain.getAuto("4NF_6")
        );
    }

}

