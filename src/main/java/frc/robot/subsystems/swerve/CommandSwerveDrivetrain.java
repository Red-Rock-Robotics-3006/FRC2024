package frc.robot.subsystems.swerve;

import java.util.function.Supplier;

import org.ejml.simple.SimpleMatrix;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentricFacingAngle;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.subsystems.swerve.generated.*;
import frc.robot.util.TalonUtils;
import frc.robot.Constants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.localization.Localization;
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

    public static final double kRejectionDistance = 3;

    private boolean useAbsolute = true;

    private AprilTagIO[] aprilTagLL;

    private FieldCentricFacingAngle angleSwerveRequest;

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
    private Field2d tagField0 = new Field2d(),
            tagField1 = new Field2d(),
            tagField2 = new Field2d(),
            tagField3 = new Field2d();

    private Pose2d tagPose0 = new Pose2d(),
            tagPose1 = new Pose2d(0, 1, new Rotation2d()),
            tagPose2 = new Pose2d(1, 0, new Rotation2d()),
            tagPose3 = new Pose2d(1, 1, new Rotation2d());

    
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

    private double rejectionDistance;


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

        this.setTargetHeading(0);
        this.seedFieldRelative(new Pose2d());

        SmartDashboard.putData("field", this.field);
        SmartDashboard.putData("tag 0 field", tagField0);
        SmartDashboard.putData("tag 1 field", tagField1);
        SmartDashboard.putData("tag 2 field", tagField2);
        SmartDashboard.putData("tag 3 field", tagField3);
        

        SmartDashboard.putNumber("predict heading pid coeff", kPredictAngleCoeff);
        SmartDashboard.putNumber("predict heading pid threshold", kPredictThreshold);
        SmartDashboard.putNumber("predict heading pid higher", kHigherPredictCoeff);

        configureAll();
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

        this.setTargetHeading(0);
        this.seedFieldRelative(new Pose2d());

        SmartDashboard.putData("field", this.field);
        SmartDashboard.putData("tag 0 field", tagField0);
        SmartDashboard.putData("tag 1 field", tagField1);
        SmartDashboard.putData("tag 2 field", tagField2);
        SmartDashboard.putData("tag 3 field", tagField3);

        SmartDashboard.putNumber("predict heading pid coeff", kPredictAngleCoeff);

        SmartDashboard.putNumber("predict heading pid coeff", kPredictAngleCoeff);
        SmartDashboard.putNumber("predict heading pid threshold", kPredictThreshold);
        SmartDashboard.putNumber("predict heading pid higher", kHigherPredictCoeff);

        configureAll();

    }

    private void configureAll(){
        // new Localization();
        aprilTagLL = Localization.getLimeLights();
        System.out.println(Localization.getLimeLights());
        // for (AprilTagIO ap : aprilTagLL){
        //     SmartDashboard.putData(ap.getName() + "-field", ap.getField2d());
        // }

        SmartDashboard.putNumber("drivetrain-rejection distance", kRejectionDistance);
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public void setAngleSwerveRequest(FieldCentricFacingAngle request){
        this.angleSwerveRequest = request;
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
         

        // if (!Constants.Settings.ABSOLUTE_LOCALIZATION) return this.getState().Pose;
        // // if (Localization.tagInVision()){
        // //     Pose2d pose = new Pose2d(
        // //         Localization.getPose().getX(),
        // //         Localization.getPose().getY(),
        // //         this.getState().Pose.getRotation()
        // //     );

        // //     this.seedFieldRelative(pose);
        // //     return pose;
        // // }
        // boolean[] tagInvisions = Localization.getTags();
        // Pose2d[] poses = Localization.getPose2ds();

        // Pose2d currPose = this.getState().Pose;

        // if (tagInvisions[0]){
        //     tagPose0 = poses[0];
        //     SmartDashboard.putBoolean("drivetrain-tag 0 found", true);
        //     if (withinRejectionDistance(currPose, poses[0])){
        //         this.addVisionMeasurement(poses[0], Timer.getFPGATimestamp());
        //         SmartDashboard.putBoolean("drivetrain-tag 0 accepted", true);
        //     }
        // } else {
        //     SmartDashboard.putBoolean("drivetrain-tag 0 accepted", false);
        //     SmartDashboard.putBoolean("drivetrain-tag 0 found", false);
        // }

        // if (tagInvisions[1]){
        //     tagPose1 = poses[1];
        //     if (withinRejectionDistance(currPose, poses[1])){
        //         this.addVisionMeasurement(poses[1], Timer.getFPGATimestamp());
        //         SmartDashboard.putBoolean("drivetrain-tag 1 accepted", true);
        //     }
        // } else {
        //     SmartDashboard.putBoolean("drivetrain-tag 1 accepted", false);
        // }

        // if (tagInvisions[2]){
        //     tagPose2 = poses[2];
        //     if (withinRejectionDistance(currPose, poses[0])){
        //         this.addVisionMeasurement(poses[0], Timer.getFPGATimestamp());
        //         SmartDashboard.putBoolean("drivetrain-tag 2 accepted", true);
        //     }
        // } else {
        //     SmartDashboard.putBoolean("drivetrain-tag 2 accepted", false);
        // }

        // if (tagInvisions[3]){
        //     tagPose3 = poses[3];
        //     if (withinRejectionDistance(currPose, poses[0])){
        //         this.addVisionMeasurement(poses[0], Timer.getFPGATimestamp());
        //         SmartDashboard.putBoolean("drivetrain-tag 3 accepted", true);
        //     }
        // } else {
        //     SmartDashboard.putBoolean("drivetrain-tag 3 accepted", false);
        // }
        
        // if (Localization.tagInVision()) return Localization.getPose();
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

    public void setAbsolute(boolean b){
        this.useAbsolute = b;
    }

    @Deprecated
    public Command resetHeading(){
        // if (Constants.Settings.ABSOLUTE_LOCALIZATION){
            return new InstantCommand(
                () -> this.seedFieldRelative(),
                this
            );
        // }
        // return new SequentialCommandGroup(
        //     new InstantCommand(() -> Shooter.getInstance().setOffset(this.getState().Pose.getRotation().getDegrees() + Shooter.getInstance().getOffset()), Shooter.getInstance()),
        //     new InstantCommand(() -> this.seedFieldRelative(
        //       new Pose2d(
        //         this.getState().Pose.getX(),
        //         this.getState().Pose.getY(),
        //         new Rotation2d()
        //       )
        //     ), this
        //   ),
        //   new InstantCommand(() -> this.setTargetHeading(0), this)
        // );
    }
    
    public Command resetFieldHeading(){
        // if (Constants.Settings.ABSOLUTE_LOCALIZATION){
        //     return new FunctionalCommand(
        //         () -> {}, 
        //         () -> {}, 
        //         (interrupted) -> {
        //             if (Localization.getTags()[0]){
        //                 this.seedFieldRelative(Localization.getPose2ds()[0]);
        //             }
        //             // else if (Localization.getTags()[1]){
        //             //     this.seedFieldRelative(Localization.getPose2ds()[1]);
        //             // }
        //             // else if (Localization.getTags()[2]){
        //             //     this.seedFieldRelative(Localization.getPose2ds()[2]);
        //             // }
        //             // else if (Localization.getTags()[3]){
        //             //     this.seedFieldRelative(Localization.getPose2ds()[3]);
        //             // }
        //         },
        //         () -> Localization.tagInVision());
        // }
        return new SequentialCommandGroup(
            // new InstantCommand(() -> Shooter.getInstance().setOffset(0), Shooter.getInstance()),
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

    public Command resetOdo(){
        return new FunctionalCommand(
                () -> {}, 
                () -> {}, 
                (interrupted) -> {
                    if (Localization.getTags()[0]){
                        this.seedFieldRelative(Localization.getPose2ds()[0]);
                    }
                    this.setTargetHeading(this.getCurrentHeadingDegrees());
                    // else if (Localization.getTags()[1]){
                    //     this.seedFieldRelative(Localization.getPose2ds()[1]);
                    // }
                    // else if (Localization.getTags()[2]){
                    //     this.seedFieldRelative(Localization.getPose2ds()[2]);
                    // }
                    // else if (Localization.getTags()[3]){
                    //     this.seedFieldRelative(Localization.getPose2ds()[3]);
                    // }
                },
                () -> Localization.tagInVision());
    }

    private Command holdAngleCommand(){
        if (angleSwerveRequest == null) return new InstantCommand(() -> System.out.println("drivetrain angle object is null"));
        return this.applyRequest(
            () -> angleSwerveRequest.withTargetDirection(new Rotation2d(Math.toRadians(this.getTargetHeading())))
        );
    }

    public Command holdAngleCommand(double seconds){
        return new ParallelRaceGroup(
            new WaitCommand(seconds),
            holdAngleCommand()
        );
    }

    public Command setTargetHeadingDegreesCommand(double degrees){
        return new InstantCommand(() -> this.setTargetHeading(degrees), this);
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

    @Deprecated
    public void configureChrp(String filename){
        for (SwerveModule module : this.Modules){
            TalonUtils.addMotor(module.getDriveMotor());
        }
        System.out.println("enable chrp?: " + TalonUtils.configureOrchestra("music/sirens.chrp"));
    }
    
    public boolean withinRejectionDistance(Pose2d p1, Pose2d p2){
        // return Math.sqrt((p1.getX() - p2.getX()) * (p1.getX() - p2.getX()) + (p1.getY() - p2.getY()) * (p1.getY() - p2.getY())) < kRejectionDistance;
        double diffX = p1.getX() - p2.getX();
        double diffY = p1.getY() - p2.getY();
        return Math.hypot(diffX, diffY) < this.rejectionDistance;
        // return true;
    }

    public void addVisionMeasurements(){
        
        for (AprilTagIO aprilTagDetector : aprilTagLL){
            // if (aprilTagDetector.getPoseEstimate() != null) aprilTagDetector.getField2d().setRobotPose(aprilTagDetector.getPoseEstimate());
            if (useAbsolute && aprilTagDetector.isValid() && withinRejectionDistance(this.getState().Pose, aprilTagDetector.getPoseEstimate())){
                SmartDashboard.putBoolean(aprilTagDetector.getName() + "-drivetrain-tag-accepts", true);
                this.addVisionMeasurement(aprilTagDetector.getPoseEstimate(), aprilTagDetector.getTimeStamp(), aprilTagDetector.getStandardDeviations());
            } else {
                SmartDashboard.putBoolean(aprilTagDetector.getName() + "-drivetrain-tag-accepts", false);
            }

        }
    }
    // public void toggleChrp(){
    //     if (orchestra.isPlaying()) orchestra.stop();
    //     else orchestra.play();
    // }

    @Override
    public void setDriveState(DriveState state){
        this.driveState = state;
    }

    @Override
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
        // if (useAbsolute){
            // boolean[] tagInvisions = Localization.getTags();
            // Pose2d[] poses = Localization.getPose2ds();

            // Pose2d currPose = this.getState().Pose;

            

            // if (tagInvisions[0]){
            //     tagPose0 = poses[0];
            //         SmartDashboard.putBoolean("drivetrain-tag 0 found", true);
            //     if (withinRejectionDistance(currPose, poses[0])){
            //         if (this.useAbsolute) this.addVisionMeasurement(poses[0], Timer.getFPGATimestamp());
            //         SmartDashboard.putBoolean("drivetrain-tag 0 accepted", true);
            //     }
            // } else {
            //     SmartDashboard.putBoolean("drivetrain-tag 0 accepted", false);
            //     SmartDashboard.putBoolean("drivetrain-tag 0 found", false);
            // }

            // if (tagInvisions[1]){
            //     tagPose1 = poses[1];
            //     if (withinRejectionDistance(currPose, poses[1])){
            //         this.addVisionMeasurement(poses[1], Timer.getFPGATimestamp());
            //         SmartDashboard.putBoolean("drivetrain-tag 1 accepted", true);
            //     }
            // } else {
            //     SmartDashboard.putBoolean("drivetrain-tag 1 accepted", false);
            // }

            // if (tagInvisions[2]){
            //     tagPose2 = poses[2];
            //     if (withinRejectionDistance(currPose, poses[0])){
            //         this.addVisionMeasurement(poses[0], Timer.getFPGATimestamp());
            //         SmartDashboard.putBoolean("drivetrain-tag 2 accepted", true);
            //     }
            // } else {
            //     SmartDashboard.putBoolean("drivetrain-tag 2 accepted", false);
            // }

            // if (tagInvisions[3]){
            //     tagPose3 = poses[3];
            //     if (withinRejectionDistance(currPose, poses[0])){
            //         this.addVisionMeasurement(poses[0], Timer.getFPGATimestamp());
            //         SmartDashboard.putBoolean("drivetrain-tag 3 accepted", true);
            //     }
            // } else {
            //     SmartDashboard.putBoolean("drivetrain-tag 3 accepted", false);
            // }
        // }

        this.addVisionMeasurements();

        this.field.setRobotPose(this.getPose());

        // this.tagField0.setRobotPose(this.tagPose0);
        // this.tagField1.setRobotPose(this.tagPose1);
        // this.tagField2.setRobotPose(this.tagPose2);
        // this.tagField3.setRobotPose(this.tagPose3);
        
        SmartDashboard.putNumber("drivetrain-rotation rate", this.getRotationRate());
        // posePublisher.set(this.getState().Pose);
        SmartDashboard.putNumber("drivetrain-field heading", this.getState().Pose.getRotation().getDegrees());

        SmartDashboard.putNumber("drivetrain-fga timestamp", Timer.getFPGATimestamp());

        SmartDashboard.putBoolean("abs localization", this.useAbsolute);

        this.rejectionDistance = SmartDashboard.getNumber("drivetrain-rejection distance", kRejectionDistance);

        
        // SmartDashboard.putNumber("drivetrain-encoder val", this.Modules[2].getDriveMotor().getRotorPosition().getValueAsDouble());
        
    }


}