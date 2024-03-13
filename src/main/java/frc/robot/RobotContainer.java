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
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterCommands;
import frc.robot.subsystems.swerve.*;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain.DriveState;;

public class RobotContainer {
  public static final double kDeadBand = 0.02;
  public static final double kRotationDeadband = 0.02;
  
  private double MaxSpeed = 4.5; 
  private double MaxAngularRate = 0.5 * Math.PI; 

  public static double kAngle = 60;
  public static final double kShooterSpeed = 1;

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
  public Intake intake = Intake.getInstance();//TODO for now
  public Index index = Index.getInstance();
  // public LED led = LED.getInstance();
  public Shooter shooter = Shooter.getInstance();
  public Climber climber = Climber.getInstance();

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
    ).onFalse(
      new InstantCommand(() -> drivetrain.setTargetHeading(drivetrain.getCurrentHeadingDegrees()), drivetrain)
    );


    angle.HeadingController.setPID(CommandSwerveDrivetrain.kHeadingP, CommandSwerveDrivetrain.kHeadingI, CommandSwerveDrivetrain.kHeadingD);
    angle.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

    // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    // joystick.b().whileTrue(drivetrain
    //     .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));
        
    joystick.start().onTrue(
      drivetrain.resetHeading()
    );
    
    RobotModeTriggers.teleop().onTrue(
        drivetrain.resetFieldHeading()
    );

    // joystick.povLeft().onTrue(
    //   new InstantCommand(() -> shooter.setHoming(true))//Shooter.encoderTarget = 0.7)
    // );

    //SHOOTER ANGLE BINDINGS

    joystick.povDown().onTrue(
      ShooterCommands.setAngle(Shooter.Positions.SUB_CENTER)//Shooter.encoderTarget = 0.8)
    );
    joystick.povUp().onTrue(
      ShooterCommands.setAngle(Shooter.Positions.SUB_LEFT)//Shooter.encoderTarget = 0.7)
    );
    joystick.povRight().onTrue(
      ShooterCommands.setAngle(Shooter.Positions.SUB_RIGHT)//Shooter.encoderTarget = 0.7)
    );

    //INTAKE PROCESS BINDINGS

    joystick.leftBumper().onTrue(
      CommandFactory.intakeCommand()//TODO for new shooter this can just be IntakeCommands.intake();
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

    //SHOOT PROCESS BINDINGS

    mechstick.x().onTrue(
      ShooterCommands.spinUp()
    );
    joystick.x().onTrue(
      IndexCommands.start()
    );
    joystick.y().onTrue(
      new InstantCommand(() -> shooter.setShooterSpeed(SmartDashboard.getNumber("shooter test speed", kShooterSpeed)), shooter)
    );
    joystick.b().onTrue(
      new SequentialCommandGroup(
        ShooterCommands.stop(),
        IndexCommands.stop()
      )
    );

    //CLIMB PROCESS BINDINGS

    climber.setDefaultCommand(
      new RunCommand(
        () -> climber.move(mechstick.getRightTriggerAxis() - mechstick.getLeftTriggerAxis()), 
        climber)
    );
    mechstick.leftBumper().whileTrue(
      new RunCommand(() -> climber.setLeftSpeed(SmartDashboard.getNumber("reset speed", Climber.kResetSpeed)), climber)
    ).onFalse(
      new InstantCommand(
        () -> {climber.stop(); climber.resetLeftEncoder();},
        climber
      )
    );
    mechstick.rightBumper().whileTrue(
      new RunCommand(() -> climber.setRightSpeed(SmartDashboard.getNumber("reset speed", Climber.kResetSpeed)), climber)
    ).onFalse(
      new InstantCommand(
        () -> {climber.stop(); climber.resetRightEncoder();},
        climber
      )
    );

    //MECH CONTROLLER SETTINGS BINDINGS
        
    mechstick.back().onTrue(
      new InstantCommand(() -> drivetrain.setDriveState(DriveState.FIELD_CENTRIC_NO_LOCK), drivetrain)
    ).onFalse(
      new InstantCommand(() -> drivetrain.setDriveState(DriveState.FIELD_CENTRIC), drivetrain)
    );
    mechstick.start().onTrue(
      drivetrain.resetFieldHeading()
    );
    mechstick.povLeft().whileTrue(
      drivetrain.applyRequest(() -> brake));



    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);  

    // mechstick.a().onTrue(
    //   new InstantCommand(() -> drivetrain.toggleChrp(), drivetrain)
    // );
    
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
    SmartDashboard.putNumber("homing d", 0.01);

    SmartDashboard.putNumber("amp speed", 0.12);

    SmartDashboard.putNumber("shoot angle", kAngle);
    SmartDashboard.putNumber("shooter test speed", kShooterSpeed);
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
  } 

  public void configurePathPlanner(){
    NamedCommands.registerCommand("IntakeCommand", CommandFactory.intakeCommand());
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

    angle.HeadingController.setP(SmartDashboard.getNumber("homing p", 4.25));
    angle.HeadingController.setD(SmartDashboard.getNumber("homing d", 0.2));
  
    SmartDashboard.putNumber("current p", angle.HeadingController.getP());
    SmartDashboard.putNumber("current d", angle.HeadingController.getD());

    this.MaxAngularRate = SmartDashboard.getNumber("max turn", CommandSwerveDrivetrain.kTurnSpeed) * Math.PI;
    this.MaxSpeed = SmartDashboard.getNumber("max speed", CommandSwerveDrivetrain.kDriveSpeed);

  }

  public void configureSelector(){
    m_chooser.setDefaultOption("no auto", Commands.print("good luck drivers!"));
    m_chooser.addOption("straight", drivetrain.getAuto("StraightLineAuto"));
    // m_chooser.addOption("four note", runAuto);
    m_chooser.addOption("alliance neutral: one note pick up mid", Autos.oneNoteGrabAuto());
    m_chooser.addOption("blue: one note source side", Autos.oneNoteSourceSide());
    m_chooser.addOption("alliance neutral: two note", Autos.twoNoteAuto());
    m_chooser.addOption("alliance neutral: two note paths", Autos.twoNotePaths());
    m_chooser.addOption("blue: source side one note to center", Autos.blueSourceToCenter());
    m_chooser.addOption("red: source side one note to center", Autos.redSourceToCenter());
    m_chooser.addOption("blue: amp side no pickup", Autos.blueAmpNoPickup());
    m_chooser.addOption("red: amp side no pickup", Autos.redAmpNoPickup());
    m_chooser.addOption("two note dont run at comp", Autos.twoNoteAuto2());
    m_chooser.addOption("just shoot sides", Autos.justShootSides());
    m_chooser.addOption("blue: troll auto", Autos.trollAuto());
    m_chooser.addOption("blue: troll auto paths", Autos.trollAutoPath());

    //DO NOT RUN THESE THREE AUTOS
    m_chooser.addOption("DO NOT RUN: tysens idea", Autos.tysensIdeaAuto());
    m_chooser.addOption("DO NOT RUN: grand prix", Autos.grandPrixAuto());
    m_chooser.addOption("DO NOT RUN: max ver", Autos.superMaxAuto());
    //DO NOT RUN THESE THREE AUTOS

    m_chooser.addOption("alliance neutral: really a fournote this time", Autos.m_4note());
    m_chooser.addOption("4note paths: dont run at comp", drivetrain.getAuto("4N_P"));
    m_chooser.addOption("4note paths not pp", Autos.m_4_1p_3w());
    m_chooser.addOption("blue: 3 note source side", drivetrain.getAuto("3NS_1B"));
    m_chooser.addOption("blue: 3 note source side alt", drivetrain.getAuto("3NS_1B_Alt"));
                                                                                                                                                                                                                                                                                                                                                                                                                m_chooser.addOption("test", Autos.test());
    
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