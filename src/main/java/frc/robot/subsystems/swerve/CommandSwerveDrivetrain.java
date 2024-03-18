package frc.robot.subsystems.swerve;

import java.util.function.Supplier;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
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
import frc.robot.util.TalonUtils;
import frc.robot.subsystems.*;
import frc.robot.subsystems.shooter.Localization;
import frc.robot.subsystems.shooter.Shooter;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem, SwerveIO {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    public static final double kPredictAngleCoeff = 0.15;
    public static final double kPredictThreshold = 1;

    public static final double kHigherPredictCoeff = 0.2;

    // private StructPublisher<Pose2d> posePublisher = NetworkTableInstance.getDefault()
    //             .getStructTopic("pose", Pose2d.struct).publish();

    // private Orchestra orchestra = new Orchestra();

    public static final TalonFXConfiguration kDriveConfig = new TalonFXConfiguration()
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(60  )
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimit(120)
                .withStatorCurrentLimitEnable(true)
            );   

    private static final TalonFXConfiguration kTurnConfig = new TalonFXConfiguration()
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(60)
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimit(120)
                .withStatorCurrentLimitEnable(true)
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
        // if (!Utils.isSimulation()){
            // for (SwerveModule module : this.Modules){
            //     module.getDriveMotor().getConfigurator().apply(kDriveConfig);
            //     module.getDriveMotor().getConfigurator().apply(TunerConstants.driveGains);


                

            // //     // TalonUtils.addMotor(module.getSteerMotor());
            // //     // module.getSteerMotor().getConfigurator().apply(kTurnConfig);
            // //     // orchestra.addInstrument(module.getDriveMotor());
            // //     // orchestra.addInstrument(module.getSteerMotor());
            // }
            // orchestra.loadMusic("music/siren.chrp");
        // }

        // System.out.println("configure orchestra: " + TalonUtils.configureOrchestra("music/sirens.chrp"));

        SmartDashboard.putData("field", this.field);

        SmartDashboard.putNumber("predict heading pid coeff", kPredictAngleCoeff);
        SmartDashboard.putNumber("predict heading pid threshold", kPredictThreshold);
        SmartDashboard.putNumber("predict heading pid higher", kHigherPredictCoeff);
    }
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        configurePathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }
        // if (!Utils.isSimulation()){
            // for (SwerveModule module : this.Modules){
            //     module.getDriveMotor().getConfigurator().apply(kDriveConfig);
            //     module.getDriveMotor().getConfigurator().apply(TunerConstants.driveGains);

            // //     // TalonUtils.addMotor(module.getSteerMotor());
            // //     // module.getSteerMotor().getConfigurator().apply(kTurnConfig);
            // //     // orchestra.addInstrument(module.getDriveMotor());
            // //     // orchestra.addInstrument(module.getSteerMotor());
            // }
            // orchestra.loadMusic("music/siren.chrp");
        // }
        // System.out.println("configure orchestra: " + TalonUtils.configureOrchestra("music/sirens.chrp"));

        SmartDashboard.putData("field", this.field);
        SmartDashboard.putNumber("predict heading pid coeff", kPredictAngleCoeff);

        SmartDashboard.putNumber("predict heading pid coeff", kPredictAngleCoeff);
        SmartDashboard.putNumber("predict heading pid threshold", kPredictThreshold);
        SmartDashboard.putNumber("predict heading pid higher", kHigherPredictCoeff);

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
            ()-> this.getPose(), // Supplier of current robot pose
            this::seedFieldRelative,  // Consumer for seeding pose against auto
            this::getCurrentRobotChassisSpeeds,
            (speeds)->this.setControl(autoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the robot
            new HolonomicPathFollowerConfig(new PIDConstants(10, 0, 0),
                                            new PIDConstants(10, 0, 0),
                                            TunerConstants.kSpeedAt12VoltsMps,
                                            driveBaseRadius,
                                            new ReplanningConfig()),
            // ()->false, // Change this if the path needs to be flipped on red vs blue
            () -> {//TODO this is is testing and we hope it works
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                  return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            this); // Subsystem for requirements
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    public Pose2d getPose(){
        if (Localization.tagInVision()){
            Pose2d pose = new Pose2d(
                Localization.getPose().getX(),
                Localization.getPose().getY(),
                this.getState().Pose.getRotation()
            );

            this.seedFieldRelative(pose);
            return pose;
        }
        
         return this.getState().Pose;
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

    public void configureCurrentLimits(){
        System.out.println(this.ModuleCount);
        for (int i = 0; i < this.ModuleCount; i++){
            this.Modules[i].getDriveMotor().getConfigurator().apply(kDriveConfig);
            if (i == 1 || i == 3){
                this.Modules[i].getDriveMotor().setInverted(true);
            }
            this.Modules[i].getDriveMotor().setNeutralMode(NeutralModeValue.Brake);
        }
    }

    public double getPredictHeadingDegrees(){
        // return this.getCurrentHeadingDegrees() + SmartDashboard.getNumber("predict heading pid coeff", kPredictAngleCoeff) * this.getRotationRate();
        // double currentSpeed = this.getRotationRate();
        // double threshold = SmartDashboard.getNumber("predict heading pid threshold", kPredictAngleCoeff);
        // double lower = SmartDashboard.getNumber("predict heading pid coeff", kPredictAngleCoeff);
        // double higher = SmartDashboard.getNumber("predict heading pid higher", kHigherPredictCoeff);

        // double extra = 0;

        // if (currentSpeed > threshold) {extra = higher;}
        // else { extra = lower;}

        // SmartDashboard.putNumber("current predict", extra);
        // SmartDashboard.putNumber("threshold", threshold);

        // return this.getCurrentHeadingDegrees() + extra * currentSpeed;

        return this.getCurrentHeadingDegrees();
    }
    public double getRotationRate(){
        return this.getPigeon2().getRate();
    }

    public void configureChrp(String filename){
        for (SwerveModule module : this.Modules){
            TalonUtils.addMotor(module.getDriveMotor());
        }
        System.out.println("enable chrp?: " + TalonUtils.configureOrchestra("music/sirens.chrp"));
    }

    // public void toggleChrp(){
    //     if (orchestra.isPlaying()) orchestra.stop();
    //     else orchestra.play();
    // }

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
        SmartDashboard.putNumber("drivetrain rotation rate", this.getRotationRate());
        // posePublisher.set(this.getState().Pose);
        SmartDashboard.putNumber("field heading", this.getState().Pose.getRotation().getDegrees());
    }


}
