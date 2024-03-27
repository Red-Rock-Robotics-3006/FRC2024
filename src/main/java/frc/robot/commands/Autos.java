package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.index.Index;
import frc.robot.subsystems.index.IndexCommands;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeCommands;
import frc.robot.subsystems.led.LEDCommands;
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

    public static Command trollAuto_b() {
        return new SequentialCommandGroup(
            CommandFactory.shootSideCommand(),
            ShooterCommands.setAngle(Positions.SUB_LEFT),
            ShooterCommands.setShooterSpeed(0.15),
            IndexCommands.start(),
            IntakeCommands.start(),
            TunerConstants.DriveTrain.getAuto("TrollAuto"),
            new WaitCommand(1),
            ShooterCommands.stop(),
            IndexCommands.stop(),
            IntakeCommands.stop()
        );
    }

    public static Command trollAutoPath() {
        return new SequentialCommandGroup(
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

    public static Command m_6note(){
        return new SequentialCommandGroup(

            //FIRST NOTE
            CommandFactory.shootCenterCommand(),
            ShooterCommands.spinUp(),

            //SECOND NOTE
            new ParallelCommandGroup(
                IntakeCommands.intake(),
                drivetrain.getAuto("6N_1")
            ),
            ShooterCommands.setHoming(true),
            new WaitCommand(0.5),
            ShooterCommands.shootAuto(),

            //THIRD NOTE
            new ParallelCommandGroup(
                IntakeCommands.intake(),
                drivetrain.getAuto("6N_2")
            ),
            ShooterCommands.setHoming(true),
            new WaitCommand(0.5),
            ShooterCommands.shootAuto(),
            
            //FOURTH NOTE
            new ParallelCommandGroup(
                IntakeCommands.intake(),
                drivetrain.getAuto("6N_3")
            ),
            ShooterCommands.setHoming(true),
            new WaitCommand(0.5),
            ShooterCommands.shootAuto(),

            //FIFTH NOTE
            new ParallelCommandGroup(
                IntakeCommands.intake(),
                new SequentialCommandGroup(
                    drivetrain.getAuto("6N_4"),
                    drivetrain.getAuto("6N_5")
                )
            ),
            ShooterCommands.setHoming(true),
            new WaitCommand(0.5),
            ShooterCommands.shootAuto(),

            //SIXTH NOTE
            new ParallelCommandGroup(
                IntakeCommands.intake(),
                new SequentialCommandGroup(
                    drivetrain.getAuto("6N_6"),
                    drivetrain.getAuto("6N_7")
                )
            ),
            ShooterCommands.setHoming(true),
            new WaitCommand(0.5),
            ShooterCommands.shootAuto(),

            //END
            ShooterCommands.stop()
        );
    }

    public static Command m_autoaim_4note(){
        return new SequentialCommandGroup(

            //FIRST NOTE
            CommandFactory.shootCenterCommand(),
            ShooterCommands.spinUp(),

            //SECOND NOTE
            new ParallelCommandGroup(
                IntakeCommands.intake(),
                drivetrain.getAuto("6N_1")
            ),
            ShooterCommands.setHoming(true),
            new WaitCommand(0.5),
            ShooterCommands.shootAuto(),

            //THIRD NOTE
            new ParallelCommandGroup(
                IntakeCommands.intake(),
                drivetrain.getAuto("6N_2")
            ),
            ShooterCommands.setHoming(true),
            new WaitCommand(0.5),
            ShooterCommands.shootAuto(),
            
            //FOURTH NOTE
            new ParallelCommandGroup(
                IntakeCommands.intake(),
                drivetrain.getAuto("6N_3")
            ),
            ShooterCommands.setHoming(true),
            new WaitCommand(0.5),
            ShooterCommands.shootAuto(),

            //END
            LEDCommands.setAutoEnd(true),
            ShooterCommands.stop()
        );
    }

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

    public static Command m_3note_blue() {
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
                    drivetrain.getAuto("4NF_5"), 
                    drivetrain.getAuto("4NF_6")
                )
            ),
            ShooterCommands.shootAuto(),

            //END
            ShooterCommands.stop()
        );
    }

    public static Command m_3note_red() {
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
                    ShooterCommands.setAngle(Positions.SUB_LEFT)
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
                    ShooterCommands.setAngle(Positions.SUB_LEFT)
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

    public static Command m_2note_1g_b() {
        return new SequentialCommandGroup(
            
            //FIRST NOTE
            CommandFactory.shootSideCommand(),
            ShooterCommands.spinUp(),

            //SECOND NOTE
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    IntakeCommands.intake(),
                    ShooterCommands.setAngle(Positions.SUB_LEFT)
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
                    IntakeCommands.intake()
                ),
                new SequentialCommandGroup(
                    drivetrain.getAuto("3N_CB_3")
                )
            ),

            //END
            ShooterCommands.stop()
        );
    }

    public static Command m_3note_r() {
        return new SequentialCommandGroup(
            
            //FIRST NOTE
            CommandFactory.shootSideCommand(),
            ShooterCommands.spinUp(),

            //SECOND NOTE
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    IntakeCommands.intake(),
                    ShooterCommands.setAngle(Positions.SUB_LEFT)
                ),
                new SequentialCommandGroup(
                    drivetrain.getAuto("3N_CR_1"), 
                    drivetrain.getAuto("3N_CR_2")
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
                    drivetrain.getAuto("3N_CR_3"), 
                    drivetrain.getAuto("3N_CR_4")
                )
            ),
            ShooterCommands.shootAuto(),

            //END
            ShooterCommands.stop()
        );
    }

    public static Command m_3note_a_r() {
        return new SequentialCommandGroup(
            // drivetrain.resetFieldHeading(),
            // new InstantCommand(
            //     () -> shooter.setOffset(60),
            //     shooter
            // ),
            //FIRST NOTE
            CommandFactory.shootSideCommand(),
            ShooterCommands.spinUp(),

            //SECOND NOTE
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    IntakeCommands.intake(),
                    ShooterCommands.setAngle(Positions.SUB_LEFT)
                ),
                new SequentialCommandGroup(
                    drivetrain.getAuto("3N_CR_1"), 
                    drivetrain.getAuto("3N_CR_2")
                )
            ),
            ShooterCommands.shootAuto(),

            //THIRD NOTE
            new ParallelCommandGroup(
                IntakeCommands.intake(),
                new SequentialCommandGroup(
                    drivetrain.getAuto("3N_CR_3"), 
                    drivetrain.getAuto("3N_CRA_4")
                )
            ),
            ShooterCommands.setHoming(true),
            new WaitCommand(0.5),
            ShooterCommands.shootAuto(),

            //END
            ShooterCommands.stop()
        );
    }

    public static Command m_autoaim_3note_r() {
        return new SequentialCommandGroup(
            ShooterCommands.spinUp(),
            
            drivetrain.getAuto("3N_CRA_1"),
            ShooterCommands.setHoming(true),
            new WaitCommand(0.5),
            ShooterCommands.shootAuto(),

            new ParallelCommandGroup(
                IntakeCommands.intake(),
                new SequentialCommandGroup(
                    drivetrain.getAuto("3N_CRA_2"), 
                    drivetrain.getAuto("3N_CRA_3")
                )
            ),
            ShooterCommands.setHoming(true),
            new WaitCommand(0.5),
            ShooterCommands.shootAuto(),

            new ParallelCommandGroup(
                IntakeCommands.intake(),
                new SequentialCommandGroup(
                    drivetrain.getAuto("3N_CRA_4"), 
                    drivetrain.getAuto("3N_CRA_5")
                )
            ),
            ShooterCommands.setHoming(true),
            new WaitCommand(0.5),
            ShooterCommands.shootAuto(),

            ShooterCommands.stop()
        );
    }

    // public static Command m_2note_1g_r() {
    //     return new SequentialCommandGroup(
            
    //         //FIRST NOTE
    //         CommandFactory.shootSideCommand(),
    //         ShooterCommands.spinUp(),

    //         //SECOND NOTE
    //         new ParallelCommandGroup(
    //             new SequentialCommandGroup(
    //                 IntakeCommands.intake(),
    //                 ShooterCommands.setAngle(Positions.SUB_LEFT)
    //             ),
    //             new SequentialCommandGroup(
    //                 drivetrain.getAuto("3N_CR_1"), 
    //                 drivetrain.getAuto("3N_CR_2")
    //             )
    //         ),
    //         ShooterCommands.shootAuto(),
    //         ShooterCommands.stop()

    //         //THIRD NOTE
    //         new ParallelCommandGroup(
    //             IntakeCommands.intake(),
    //             drivetrain.getAuto("3N_CR_3")
    //         )
    //         //END
    //     );
    // }

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

