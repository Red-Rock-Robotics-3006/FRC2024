package frc.robot.subsystems.swerve;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.swerve.generated.*;
import frc.robot.subsystems.*;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem, SwerveIO {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private static final TalonFXConfiguration kDriveConfig = new TalonFXConfiguration()
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(60)
                .withSupplyCurrentLimitEnable(true)
            );   

    private static final TalonFXConfiguration kTurnConfig = new TalonFXConfiguration()
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(40)
                .withSupplyCurrentLimitEnable(true)
            );   

    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();

    public static PathConstraints kConstraints = new PathConstraints(
        2, 1, 1.5, 1.5);

    private double targetHeadingDegrees = 0;

    private Field2d field = new Field2d();

    public static final double kDriveSpeed = 5.0;
    public static final double kTurnSpeed = 2;

    public enum DriveState{
        FIELD_CENTRIC,
        ROBOT_CENTRIC,
        FIELD_CENTRIC_NO_LOCK
    }

    private DriveState driveState = DriveState.FIELD_CENTRIC;


    public static final double
        kHeadingP = 4.25,
        kHeadingI = 0,
        kHeadingD = 0.2;


    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        configurePathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }

        for (SwerveModule module : this.Modules){
            module.getDriveMotor().getConfigurator().apply(kDriveConfig);
            module.getSteerMotor().getConfigurator().apply(kTurnConfig);
        }

        SmartDashboard.putData("field", this.field);
    }
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        configurePathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }
        for (SwerveModule module : this.Modules){
            module.getDriveMotor().getConfigurator().apply(kDriveConfig);
            module.getSteerMotor().getConfigurator().apply(kTurnConfig);
        }
        SmartDashboard.putData("field", this.field);
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    private void configurePathPlanner() {
        double driveBaseRadius = 0;
        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }

        AutoBuilder.configureHolonomic(
            ()->this.getState().Pose, // Supplier of current robot pose
            this::seedFieldRelative,  // Consumer for seeding pose against auto
            this::getCurrentRobotChassisSpeeds,
            (speeds)->this.setControl(autoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the robot
            new HolonomicPathFollowerConfig(new PIDConstants(10, 0, 0),
                                            new PIDConstants(10, 0, 0),
                                            TunerConstants.kSpeedAt12VoltsMps,
                                            driveBaseRadius,
                                            new ReplanningConfig()),
            // ()->false, // Change this if the path needs to be flipped on red vs blue
            // () -> {
            //     var alliance = DriverStation.getAlliance();
            //     if (alliance.isPresent()){
            //         return alliance.get() == Alliance.Red;
            //     }
            //     return false;
            // },
            () -> false,
            this); // Subsystem for requirements
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    public Command getAuto(String text){
        return new PathPlannerAuto(text);
    }

    public Command resetHeading(){
        return new SequentialCommandGroup(
            new InstantCommand(() -> Shooter.getInstance().setOffset(this.getState().Pose.getRotation().getDegrees() + Shooter.getInstance().getOffset()), Shooter.getInstance()),
            new InstantCommand(() -> this.seedFieldRelative(
              new Pose2d(
                this.getState().Pose.getX(),
                this.getState().Pose.getY(),
                new Rotation2d()
              )
            ), this
          ),
          new InstantCommand(() -> this.setTargetHeading(0), this)
        );
    }
    
    public Command resetFieldHeading(){
        return new SequentialCommandGroup(
            new InstantCommand(() -> Shooter.getInstance().setOffset(0), Shooter.getInstance()),
            new InstantCommand(() -> this.seedFieldRelative(
              new Pose2d(
                this.getState().Pose.getX(),
                this.getState().Pose.getY(),
                new Rotation2d()
              )
            ), this
          ),
          new InstantCommand(() -> this.setTargetHeading(0), this)
        );
    }

    public Command goToPose(Pose2d pose){
        return AutoBuilder.pathfindToPose(pose, kConstraints);
    }
    public double getRotationRate(){
        return this.getPigeon2().getRate();
    }

    @Override
    public void setDriveState(DriveState state){
        this.driveState = state;
    }

    public DriveState getDriveState(){
        return this.driveState;
    }


    

    @Override
    public void setTargetHeading(double degrees){
        this.targetHeadingDegrees = degrees;
    }

    @Override
    public double getCurrentHeadingDegrees(){
        return this.getState().Pose.getRotation().getDegrees();
    }

    @Override
    public double getTargetHeading(){
        return this.targetHeadingDegrees;
    }

    @Override
    public void periodic(){
        this.field.setRobotPose(this.getState().Pose);
        SmartDashboard.putNumber("field heading", this.getState().Pose.getRotation().getDegrees());
    }


}
