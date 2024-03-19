package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.index.Index;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeCommands;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterCommands;
import frc.robot.subsystems.shooter.Shooter.Positions;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.generated.TunerConstants;

public class Autos {

    public static Intake intake = Intake.getInstance();
    public static Index index = Index.getInstance();
    public static Shooter shooter = Shooter.getInstance();
    public static CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;

    public static double kSpinUpTime = 2;
    public static double kShootTime = 2;

    // public static Command oneNoteGrabAuto(){
    //     return new SequentialCommandGroup(
    //         new InstantCommand(() -> TunerConstants.DriveTrain.seedFieldRelative(new Pose2d(1.34, 5.53, new Rotation2d())), TunerConstants.DriveTrain),
    //         new InstantCommand(() -> shooter.presetShoot(Shooter.Positions.SUB_LEFT), shooter),
    //         new InstantCommand(() -> shooter.setShooterSpeed(1), shooter),
    //         new WaitCommand(2.5),
    //         new InstantCommand(
    //             () -> index.startTransfer(),
    //             index
    //         ),
    //         new WaitCommand(2),
    //         new InstantCommand(
    //             () -> {
    //                 index.stopTransfer();
    //                 shooter.setShooterSpeed(0);
    //             },
    //             index,
    //             shooter
    //         ),
    //         new InstantCommand(
    //             () -> {
    //                 intake.setHoming(true);
    //             }
    //         ),
    //         TunerConstants.DriveTrain.getAuto("MidNote")
    //     );
    // }
    
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
            // new InstantCommand(
            //     () -> {
            //         intake.setHoming(true);
            //     }
            // ),
            // TunerConstants.DriveTrain.getAuto("MidNote")
            new ParallelCommandGroup(
                IntakeCommands.intake(),
                TunerConstants.DriveTrain.getAuto("MidNote")
            )
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
            IntakeCommands.intake(),
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
            IntakeCommands.intake(),
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
            new InstantCommand(() -> intake.startIntake(), intake),
            new InstantCommand(() -> shooter.setShooterSpeed(0.3), shooter),
            TunerConstants.DriveTrain.getAuto("TrollAuto"),
            new InstantCommand(() -> intake.stopIntake(), intake)
        );
    }

    public static Command trollAutoPath() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> TunerConstants.DriveTrain.seedFieldRelative(new Pose2d(1.34, 5.53, new Rotation2d())), TunerConstants.DriveTrain),
            TunerConstants.DriveTrain.getAuto("TrollAuto")

        );
    }

    public static Command grandPrixAuto() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> TunerConstants.DriveTrain.seedFieldRelative(new Pose2d(1.34, 5.53, new Rotation2d())), TunerConstants.DriveTrain),
            TunerConstants.DriveTrain.getAuto("CrescendoGrandPrixAuto")

        );
    }

    public static Command tysensIdeaAuto() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> TunerConstants.DriveTrain.seedFieldRelative(new Pose2d(1.34, 5.53, new Rotation2d())), TunerConstants.DriveTrain),
            TunerConstants.DriveTrain.getAuto("TysensIdeaAuto")

        );
    }

    public static Command superMaxAuto() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> TunerConstants.DriveTrain.seedFieldRelative(new Pose2d(1.34, 5.53, new Rotation2d())), TunerConstants.DriveTrain),
            TunerConstants.DriveTrain.getAuto("TUTUTUDUMAXVERSTAPPENAuto")

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
            drivetrain.getAuto("4NFC_1"),
            drivetrain.getAuto("4NFC_2"),
            drivetrain.getAuto("4NFC_3"),
            drivetrain.getAuto("4NFC_4"),
            drivetrain.getAuto("4NFC_5"),
            drivetrain.getAuto("4NFC_6")
        );
    }

    public static final double kWaitTime = 0.5;

    public static Command m_4note(){
        return new SequentialCommandGroup(

            //FIRST NOTE
            CommandFactory.shootCenterCommand(),
            ShooterCommands.spinUp(),

            //SECOND NOTE
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    IntakeCommands.intake(),
                    ShooterCommands.setAngle(Positions.SUB_LEFT)
                ),
                new SequentialCommandGroup(
                    drivetrain.getAuto("4NF_1"),
                    drivetrain.getAuto("4NF_2")
                )
            ),
            ShooterCommands.shootAuto(),

            //THIRD NOTE
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    IntakeCommands.intake(),
                    ShooterCommands.setAngle(Positions.SUB_LEFT)
                ),
                new SequentialCommandGroup(
                    drivetrain.getAuto("4NF_3"),
                    drivetrain.getAuto("4NF_4")
                )
            ),
            ShooterCommands.shootAuto(),
            
            //FOURTH NOTE
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    IntakeCommands.intake(),
                    ShooterCommands.setAngle(Positions.SUB_LEFT)
                ),
                new SequentialCommandGroup(
                    drivetrain.getAuto("4NF_5"), 
                    drivetrain.getAuto("4NF_6")
                )
            ),
            ShooterCommands.shootAuto(),

            //END
            ShooterCommands.stop()
        );
    }

    public static Command m_3note_paths() {
        return new SequentialCommandGroup(
            drivetrain.getAuto("3N_CB_1"), 
            drivetrain.getAuto("3N_CB_2"),
            drivetrain.getAuto("3N_CB_3"), 
            drivetrain.getAuto("3N_CB_4")
        );
    }

    public static Command m_3note_b() {
        return new SequentialCommandGroup(
            
            //FIRST NOTE
            CommandFactory.shootSideCommand(),
            ShooterCommands.spinUp(),

            //SECOND NOTE
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    IntakeCommands.intake(),
                    ShooterCommands.setAngle(Positions.AUTO_SIDES)
                ),
                new SequentialCommandGroup(
                    drivetrain.getAuto("3N_CB_1"), 
                    drivetrain.getAuto("3N_CB_2")
                )
            ),
            ShooterCommands.shootAuto(),

            //THIRD NOTE
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    IntakeCommands.intake(),
                    ShooterCommands.setAngle(Positions.AUTO_SIDES)
                ),
                new SequentialCommandGroup(
                    drivetrain.getAuto("3N_CB_3"), 
                    drivetrain.getAuto("3N_CB_4")
                )
            ),
            ShooterCommands.shootAuto(),

            //END
            ShooterCommands.stop()
        );
    }

    public static Command m_3note_alt_b() {
        return new SequentialCommandGroup(
            
            //FIRST NOTE
            CommandFactory.shootSideCommand(),
            ShooterCommands.spinUp(),

            //SECOND NOTE
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    IntakeCommands.intake(),
                    ShooterCommands.setAngle(Positions.AUTO_SIDES)
                ),
                new SequentialCommandGroup(
                    drivetrain.getAuto("3N_CB_1"), 
                    drivetrain.getAuto("3N_CB_2")
                )
            ),
            ShooterCommands.shootAuto(),

            //THIRD NOTE
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    IntakeCommands.intake(),
                    ShooterCommands.setAngle(Positions.AUTO_SIDES)
                ),
                new SequentialCommandGroup(
                    drivetrain.getAuto("3N_WB_3"), 
                    drivetrain.getAuto("3N_WB_4")
                )
            ),
            ShooterCommands.shootAuto(),

            //END
            ShooterCommands.stop()
        );
    }

    public static Command m_2note_1grab_b() {
        return new SequentialCommandGroup(
            
            //FIRST NOTE
            CommandFactory.shootSideCommand(),
            ShooterCommands.spinUp(),

            //SECOND NOTE
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    IntakeCommands.intake(),
                    ShooterCommands.setAngle(Positions.AUTO_SIDES)
                ),
                new SequentialCommandGroup(
                    drivetrain.getAuto("3N_CB_1"), 
                    drivetrain.getAuto("3N_CB_2")
                )
            ),
            ShooterCommands.shootAuto(),

            //THIRD NOTE (grab)
            new ParallelCommandGroup(
                IntakeCommands.intake(),
                drivetrain.getAuto("3N_CB_3")
            ),

            //END
            ShooterCommands.stop()
        );
    }

}

