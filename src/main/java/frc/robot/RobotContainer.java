// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.swerve.generated.TunerConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.CommandFactory;
import frc.robot.subsystems.*;
import frc.robot.subsystems.index.Index;
import frc.robot.subsystems.index.IndexCommands;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeCommands;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.led.LEDCommands;
import frc.robot.subsystems.led.State;
import frc.robot.subsystems.localization.Localization;
import frc.robot.subsystems.shooter.*;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.*;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain.DriveState;;

public class RobotContainer {
  public static final double kDeadBand = 0.02;
  public static final double kRotationDeadband = 0.02;
  
  private double MaxSpeed = 4.5; 
  private double MaxAngularRate = 0.5 * Math.PI; 

  public static double kAngle = 60;
  public static final double kShooterSpeed = 0.5;

  private final CommandXboxController joystick = new CommandXboxController(0);
  private final CommandXboxController mechstick = new CommandXboxController(1);
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * kDeadBand).withRotationalDeadband(MaxAngularRate * kRotationDeadband)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); 

  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  
  private final SwerveRequest.FieldCentricFacingAngle angle = new SwerveRequest.FieldCentricFacingAngle()
      .withDeadband(MaxSpeed * kDeadBand)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final SwerveRequest.RobotCentric driveRobotCentric = new SwerveRequest.RobotCentric()
      .withDeadband(MaxSpeed * kDeadBand)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);


  private final Telemetry logger = new Telemetry(MaxSpeed);
  // private PowerDistributionHub powerDistributionHub = PowerDistributionHub.getInstance();


  public Intake intake = Intake.getInstance();//TODO for now
  public Index index = Index.getInstance();
  public LED led = LED.getInstance();
  public Shooter shooter = Shooter.getInstance();
  public Climber climber = Climber.getInstance();
  public Localization localization = new Localization();

  private double targetHeadingD = 0;

  private SendableChooser<Command> m_chooser = new SendableChooser<>();
  private void configureBindings() {

    // drivetrain.setDefaultCommand(
    //     drivetrain.applyRequest(
    //       () -> {
    //               if (Math.abs(joystick.getRightX()) > kRotationDeadband) {
    //                 SmartDashboard.putBoolean("facing angle", false);
    //                 return drive.withVelocityX(mapJoystick(-joystick.getLeftX(), -joystick.getLeftY() - 0.2)[1] * MaxSpeed)
    //                             .withVelocityY(mapJoystick(-joystick.getLeftX(), -joystick.getLeftY())[0] * MaxSpeed)
    //                             .withRotationalRate(-joystick.getRightX() * MaxAngularRate);
    //               }
    //               else {
    //                 SmartDashboard.putBoolean("facing angle", true);
    //                 return angle.withVelocityX(mapJoystick(-joystick.getLeftX(), -joystick.getLeftY())[1] * MaxSpeed)
    //                             .withVelocityY(mapJoystick(-joystick.getLeftX(), -joystick.getLeftY())[0] * MaxSpeed)
    //                             .withTargetDirection(new Rotation2d(Math.toRadians(drivetrain.getTargetHeading())));   
    //               }    
    //       }
    //     ) 
    // );
    
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(
          () -> {
                  if (drivetrain.getDriveState() == DriveState.ROBOT_CENTRIC){
                    SmartDashboard.putBoolean("facing angle", false);
                    SmartDashboard.putBoolean("robot centric", true);
                    return driveRobotCentric.withVelocityX(mapJoystick(-joystick.getLeftX(), -joystick.getLeftY())[1] * MaxSpeed)
                                            .withVelocityY(mapJoystick(-joystick.getLeftX(), -joystick.getLeftY())[0] * MaxSpeed)
                                            .withRotationalRate(-joystick.getRightX() * MaxAngularRate);
                  }
                  else if (drivetrain.getDriveState() == DriveState.FIELD_CENTRIC_NO_LOCK || Math.abs(joystick.getRightX()) > kRotationDeadband) {
                    SmartDashboard.putBoolean("facing angle", false);
                    SmartDashboard.putBoolean("robot centric", false);

                    return drive.withVelocityX(mapJoystick(-joystick.getLeftX(), -joystick.getLeftY())[1] * MaxSpeed)
                                .withVelocityY(mapJoystick(-joystick.getLeftX(), -joystick.getLeftY())[0] * MaxSpeed)
                                .withRotationalRate(-joystick.getRightX() * MaxAngularRate);
                  }
                  else {
                    SmartDashboard.putBoolean("facing angle", true);
                    SmartDashboard.putBoolean("robot centric", false);


                    return angle.withVelocityX(mapJoystick(-joystick.getLeftX(), -joystick.getLeftY())[1] * MaxSpeed)
                                .withVelocityY(mapJoystick(-joystick.getLeftX(), -joystick.getLeftY())[0] * MaxSpeed)
                                .withTargetDirection(new Rotation2d(Math.toRadians(drivetrain.getTargetHeading())));   
                  }    
          }
        ) 
    );
    new Trigger(
      () -> Math.abs(joystick.getRightX()) > kRotationDeadband
    ).onTrue(
            new FunctionalCommand(
        () -> {}, () -> {drivetrain.setTargetHeading(drivetrain.getCurrentHeadingDegrees());}, (interrupted) -> {drivetrain.setTargetHeading(drivetrain.getCurrentHeadingDegrees());}, 
        () -> {return Math.abs(joystick.getRightX()) < kRotationDeadband && Math.abs(drivetrain.getRotationRate()) < SmartDashboard.getNumber("predict heading pid threshold", CommandSwerveDrivetrain.kPredictThreshold);})
    );  


    angle.HeadingController.setPID(CommandSwerveDrivetrain.kHeadingP, CommandSwerveDrivetrain.kHeadingI, CommandSwerveDrivetrain.kHeadingD);
    angle.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

    // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    // joystick.b().whileTrue(drivetrain
    //     .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));
        
    joystick.start().onTrue(
      drivetrain.resetHeading()
    );

    joystick.back().onTrue(
      drivetrain.resetFieldHeading()
    );
    
    RobotModeTriggers.teleop().onTrue(
      new SequentialCommandGroup(
        new InstantCommand(() -> drivetrain.setAbsolute(false)),
        drivetrain.resetFieldHeading()
      )
        // drivetrain.resetFieldHeading()
    // );
    );

    RobotModeTriggers.disabled().onTrue(
      new InstantCommand(() -> drivetrain.setAbsolute(true))
    );

    // RobotModeTriggers.autonomous().onTrue(
    //   drivetrain.resetFieldHeading()
    // );

    //SHOOTER ANGLE BINDINGS

    joystick.povDown().onTrue(
      new InstantCommand(
        () -> {shooter.deny(); System.out.println("povdown");},
        shooter
      )
      // ShooterCommands.setAngle(Shooter.Positions.SUB_CENTER)//Shooter.encoderTarget = 0.8)
    );
    joystick.povUp().onTrue(
      new InstantCommand(
        () -> {shooter.accept(); System.out.println("povup");},
        shooter
      )
      // ShooterCommands.setAngle(Shooter.Positions.SUB_LEFT)//Shooter.encoderTarget = 0.7)
    );
    joystick.povRight().onTrue(
      new InstantCommand(
        () -> shooter.setTarget(SmartDashboard.getNumber("amp angle", Shooter.kAmpAngle)),
        shooter
      )
    );
    joystick.leftTrigger(0.25).whileTrue(
      ShooterCommands.setAngle(Shooter.Positions.SUB_LEFT)//Shooter.encoderTarget = 0.7)
    ).onFalse(
      ShooterCommands.setAngle(Shooter.Positions.INTAKE)
    );

    //INTAKE PROCESS BINDINGS

    joystick.leftBumper().onTrue(
      new SequentialCommandGroup(
        IntakeCommands.intake()
      )
    );
    joystick.rightBumper().onTrue(
      new SequentialCommandGroup(
        IntakeCommands.stop(),
        IndexCommands.retract()
      )
    );
    joystick.a().onTrue(
      new SequentialCommandGroup(
        IntakeCommands.reverse(),
        IndexCommands.reverse()
      )
    );
    mechstick.a().onTrue(
      new SequentialCommandGroup(
        IntakeCommands.start(),
        IndexCommands.start()
      )
    );

    //SHOOT PROCESS BINDINGS

    mechstick.x().onTrue(
      ShooterCommands.spinUp()
    );
    mechstick.y().onTrue(
      new SequentialCommandGroup(
        ShooterCommands.ampSpinUp(),
        LEDCommands.setState(State.SCORING_AMP)
      )
    );
    joystick.x().onTrue(
      ShooterCommands.shoot()
    );
    joystick.y().onTrue(
      new InstantCommand(() -> shooter.setShooterSpeed(SmartDashboard.getNumber("shooter test speed", kShooterSpeed)), shooter)
    );
    joystick.b().onTrue(
      new SequentialCommandGroup(
        ShooterCommands.stow(),
        ShooterCommands.stop(),
        IndexCommands.stop(),
        IntakeCommands.stop(),
        ShooterCommands.setHoming(false),
        LEDCommands.setState(State.RESTING),
        LEDCommands.setIsAmping(false)
      )
    );
    joystick.rightTrigger(0.25).whileTrue(
        ShooterCommands.setHoming(true)
    ).whileFalse(
        ShooterCommands.setHoming(false)
    );
    mechstick.povLeft().onTrue(
      new SequentialCommandGroup(
        ShooterCommands.setShooterSpeed(SmartDashboard.getNumber("lob shot speed", 0.33)),
        new InstantCommand(
          () -> shooter.setTarget(SmartDashboard.getNumber("lob shot angle", 20)),
          shooter
        )
      )
    );

    mechstick.povRight().onTrue(
      ShooterCommands.fullCourtLob()
    );

    mechstick.povUp().onTrue(
      // ShooterCommands.increaseDistanceFeed()
      ShooterCommands.setTarget(shooter.getTarget() + 0.5)
    );
    mechstick.povDown().onTrue(
      // ShooterCommands.decreaseDistanceFeed()
      ShooterCommands.setTarget(shooter.getTarget() - 0.5)
    );
    mechstick.rightStick().onTrue(
      new InstantCommand(
        () -> {shooter.exportTable(); System.out.println("mechstick rightStick export table pressed");},
        shooter
      )
    );

    //CLIMB PROCESS BINDINGS

    // climber.setDefaultCommand(
    //   new RunCommand(
    //     () -> climber.move(mechstick.getRightTriggerAxis() - mechstick.getLeftTriggerAxis()), 
    //     climber)
    // );

    // climber.setDefaultCommand(
    //   new RunCommand(
    //     () -> {climber.moveLeft(mechstick.getLeftTriggerAxis()); climber.moveRight(mechstick.getRightTriggerAxis());}, 
    //     climber)
    // );
    // mechstick.leftBumper().whileTrue(
    //   new RunCommand(() -> climber.setLeftSpeed(SmartDashboard.getNumber("reset speed", Climber.kResetSpeed)), climber)
    // ).onFalse(
    //   new InstantCommand(
    //     () -> {climber.stop(); climber.resetLeftEncoder();},
    //     climber
    //   )
    // );
    // mechstick.rightBumper().whileTrue(
    //   new RunCommand(() -> climber.setRightSpeed(SmartDashboard.getNumber("reset speed", Climber.kResetSpeed)), climber)
    // ).onFalse(
    //   new InstantCommand(
    //     () -> {climber.stop(); climber.resetRightEncoder();},
    //     climber
    //   )
    // );

    climber.setDefaultCommand(
      new RunCommand(
        () -> {climber.moveLeft((mechstick.leftBumper().getAsBoolean()) ? SmartDashboard.getNumber("reset speed", Climber.kResetSpeed) : 0); climber.moveRight((mechstick.rightBumper().getAsBoolean()) ? SmartDashboard.getNumber("reset speed", Climber.kResetSpeed) : 0);}, 
        climber)
    );
    new Trigger(
      () -> mechstick.getLeftTriggerAxis() > 0.02 || mechstick.getRightTriggerAxis() > 0.02
    ).whileTrue(
      new RunCommand(() -> {climber.setLeftSpeed(-mechstick.getLeftTriggerAxis());
            climber.setRightSpeed(-mechstick.getRightTriggerAxis());}, climber)
    ).onFalse(
      new InstantCommand(
        () -> {climber.stop(); climber.resetLeftEncoder();},
        climber
      )
    );

    // new Trigger(
    //   () -> mechstick.getRightTriggerAxis() > 0.02
    // ).whileTrue(
    //   new RunCommand(() -> climber.setRightSpeed(-mechstick.getRightTriggerAxis()), climber)
    // ).onFalse(
    //   new InstantCommand(
    //     () -> {climber.stop(); climber.resetRightEncoder();},
    //     climber
    //   )
    // );


    //MECH CONTROLLER SETTINGS BINDINGS
        
    mechstick.back().onTrue(
      new InstantCommand(() -> drivetrain.setDriveState(DriveState.FIELD_CENTRIC_NO_LOCK), drivetrain)
    ).onFalse(
      new InstantCommand(() -> drivetrain.setDriveState(DriveState.FIELD_CENTRIC), drivetrain)
    );
    // mechstick.start().onTrue(
    //   drivetrain.resetOdo()
    // );
    mechstick.b().whileTrue(
      drivetrain.applyRequest(() -> brake)
    );

    //LED BINDINGS

    // mechstick.povLeft().onTrue(
    //   LEDCommands.setPoliceMode(0)
    // );
    // mechstick.povUp().onTrue(
    //   LEDCommands.setPoliceMode(1)

    // );
    // mechstick.povRight().onTrue(
    //   LEDCommands.setPoliceMode(2)

    // );
    // mechstick.povDown().onTrue(
    //   LEDCommands.togglePoliceMode()
    // );

    // mechstick.povDown().onTrue(
    //   new InstantCommand(() -> TalonUtils.play())
    // );

    // mechstick.povUp().onTrue(
    //   new InstantCommand(() -> TalonUtils.stop())
    // );

    // mechstick.povDown().onTrue(
    //   new InstantCommand(() -> TalonUtils.stop())
    // );

    //TEST BINDINGS

    // shooter.setDefaultCommand(
    //   new RunCommand(
    //     () -> shooter.setAngleSpeed((joystick.getRightTriggerAxis() - joystick.getLeftTriggerAxis()) / 10), 
    //     shooter)
    // );

    // mechstick.a().onTrue(
    //   new InstantCommand(() -> drivetrain.toggleChrp(), drivetrain)
    // );


    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);  

    
  }

  /** 
   * Sets up desired values to be displayed to the Smart Dashboard
   */
  public void configureDashboard(){
    SmartDashboard.putNumber("max speed", CommandSwerveDrivetrain.kDriveSpeed);
    SmartDashboard.putNumber("max turn", CommandSwerveDrivetrain.kTurnSpeed);

    SmartDashboard.putNumber("heading p", 4.25);
    SmartDashboard.putNumber("heading d", 0.2);

    SmartDashboard.putNumber("homing p", 12);
    SmartDashboard.putNumber("homing d", 0.2);

    SmartDashboard.putNumber("amp speed", 0.12);

    SmartDashboard.putNumber("shoot angle", kAngle);
    SmartDashboard.putNumber("shooter test speed", kShooterSpeed);

    SmartDashboard.putNumber("roll forward time", Intake.kRollForwardTime);

    SmartDashboard.putNumber("lob shot speed", .33);
    SmartDashboard.getNumber("lob shot angle", 20);
  }

  public RobotContainer() {
    configureDashboard();
    configureBindings();    
    configureSelector();
    configurePathPlanner();
    SmartDashboard.putNumber("kF", Shooter.kFinalF);
    SmartDashboard.putNumber("kP", Shooter.kFinalP); // -1.1
    SmartDashboard.putNumber("encoder target", 0.7);
    SmartDashboard.putNumber("shooter target", 45);


    drivetrain.configureCurrentLimits();

    // drivetrain.configureChrp("music/sirens.chrp");
  } 

  public void configurePathPlanner(){
    NamedCommands.registerCommand("IntakeCommand", IntakeCommands.intake());
    NamedCommands.registerCommand("StartShootCommand", CommandFactory.shootCenterCommand());
    NamedCommands.registerCommand("ShootCommand", ShooterCommands.shoot());
    NamedCommands.registerCommand("SpinUpCommand", ShooterCommands.spinUp());
    NamedCommands.registerCommand("StartShootSidesCommand", CommandFactory.shootSideCommand());
  }

  public Command getAutonomousCommand() {
    // return Commands.print("No autonomous command configured");  
    return m_chooser.getSelected();
    }

  public double getDriveHeading(){
    return this.drivetrain.getState().Pose.getRotation().getDegrees();
  }

  public double getTargetHeading(){

    return this.targetHeadingD;
  }

  /**
   * Updates values needed from the dashboard
   * Called in the main Robot class
   */
  public void updateDashboard(){
    if (Constants.Settings.SHOOTER_HOMING_ENABLED && shooter.getHoming()) {
      angle.HeadingController.setP(SmartDashboard.getNumber("homing p", 12));
      angle.HeadingController.setD(SmartDashboard.getNumber("homing d", 0.2));
    }
    else {
      angle.HeadingController.setP(SmartDashboard.getNumber("heading p", 4.25));
      angle.HeadingController.setD(SmartDashboard.getNumber("heading d", 0.2));
    }
  
    SmartDashboard.putNumber("current p", angle.HeadingController.getP());
    SmartDashboard.putNumber("current d", angle.HeadingController.getD());

    this.MaxAngularRate = SmartDashboard.getNumber("max turn", CommandSwerveDrivetrain.kTurnSpeed) * Math.PI;
    this.MaxSpeed = SmartDashboard.getNumber("max speed", CommandSwerveDrivetrain.kDriveSpeed);

  }

  public void configureSelector(){
    m_chooser.setDefaultOption("no auto", Commands.print("good luck drivers!"));

    m_chooser.addOption("SIX NOTE", Autos.m_6note());
    m_chooser.addOption("SIX NOTE PATHS", Autos.m_6note_paths());
    m_chooser.addOption("SIX NOTE ALT", Autos.m_6note_alt());
    m_chooser.addOption("SIX NOTE ALT PATHS", Autos.m_6note_alt_paths());
    m_chooser.addOption("THREE NOTE SOURCE", Autos.m_3note());
    m_chooser.addOption("THREE NOTE SOURCE PATHS", Autos.m_3note_paths());
    m_chooser.addOption("AUTOAIM 4 NOTE", Autos.m_autoaim_4note());
    m_chooser.addOption("FOUR NOTE", Autos.m_4note());
    m_chooser.addOption("troll auto", Autos.m_trollauto());
    m_chooser.addOption("troll auto paths", Autos.m_trollauto_paths());
    m_chooser.addOption("FOUR NOTE AUTO AIM TEST PATHS", Autos.m_4note_autoaim_test_paths());
    
    // m_chooser.addOption("AUTOAIM 3 NOTE", Autos.m_3note());

    // m_chooser.addOption("BLUE 3 NOTE", Autos.m_3note_blue());
    // m_chooser.addOption("RED 3 NOTE", Autos.m_3note_red());

    // m_chooser.addOption("blue: troll auto", Autos.trollAuto_b());
    // m_chooser.addOption("blue: troll auto paths", Autos.trollAutoPath());

    // m_chooser.addOption("4note paths not pp", Autos.m_4_1p_3w());
    // m_chooser.addOption("blue: 3 note source side", Autos.m_3note_b());
    // m_chooser.addOption("blue: 2 note 1 grab source side", Autos.m_2note_1g_b());
    // m_chooser.addOption("blue: 3 note source side alt", drivetrain.getAuto("3NS_1B_Alt"));
    // m_chooser.addOption("three note paths", Autos.m_3note_paths());
    // m_chooser.addOption("red: 3 note source side", Autos.m_3note_r());
    // m_chooser.addOption("red: auto aim 3 note source side", Autos.m_autoaim_3note_r());
                                                                                                                                                                                                                                                                                                                                                                                                       
    // m_chooser.addOption("test", Autos.test());
    
    SmartDashboard.putData("auto chooser", m_chooser);
  }

  /**
   * Re-maps a standard controller joystick input to be more circular and rotationally uniform
   * 
   * @param x
   * @param y
   * @return array of new x and y values
   */
  public static double[] mapJoystick(double x, double y){
    // double m_x = x * Math.sqrt(1 - y * y / 2);
    // double m_y = y * Math.sqrt(1 - x * x / 2);
    // double[] converted = {m_x, m_y};
    // return converted;
    return new double[]{x, y};
  }
}