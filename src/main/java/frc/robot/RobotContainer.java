// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

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
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.swerve.generated.TunerConstants;
import frc.robot.commands.Autos;
import frc.robot.subsystems.*;
import frc.robot.subsystems.swerve.*;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain.DriveState;;

public class RobotContainer {
  public static final double kDeadBand = 0.02;
  public static final double kRotationDeadband = 0.02;
  
  private double MaxSpeed = 4.5; 
  private double MaxAngularRate = 0.5 * Math.PI; 

  public static double kAngle = 51.7;

  private final CommandXboxController joystick = new CommandXboxController(0);
  private final CommandXboxController mechstick = new CommandXboxController(1);
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * kDeadBand).withRotationalDeadband(MaxAngularRate * kRotationDeadband)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); 

  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  
  private final SwerveRequest.FieldCentricFacingAngle angle = new SwerveRequest.FieldCentricFacingAngle()
      .withDeadband(MaxSpeed * kDeadBand)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final SwerveRequest.RobotCentric driveRobotCentric = new SwerveRequest.RobotCentric()
      .withDeadband(MaxSpeed * kDeadBand)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);


  private final Telemetry logger = new Telemetry(MaxSpeed);
  public Intake intake = Intake.getInstance();//TODO for now
  public Index index = Index.getInstance();
  public LED led = LED.getInstance();
  public Shooter shooter = Shooter.getInstance();
  public Climber climber = Climber.getInstance();

  private double targetHeadingD = 0;

  private Command runAuto = drivetrain.getAuto("fournote");

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

    joystick.povDown().onTrue(
      new InstantCommand(() -> shooter.presetShoot(Shooter.Positions.SUB_CENTER))//Shooter.encoderTarget = 0.8)
    );

    joystick.povUp().onTrue(
      new InstantCommand(() -> shooter.presetShoot(Shooter.Positions.SUB_LEFT))//Shooter.encoderTarget = 0.7)
    );

    // joystick.povLeft().onTrue(
    //   new InstantCommand(() -> shooter.setHoming(true))//Shooter.encoderTarget = 0.7)
    // );

    joystick.povRight().onTrue(
      new InstantCommand(() -> shooter.presetShoot(Shooter.Positions.SUB_RIGHT))//Shooter.encoderTarget = 0.7)
    );


    joystick.leftBumper().onTrue(
      new InstantCommand(() -> {intake.toggleHoming(); System.out.println("left bumper");}, intake).andThen(new InstantCommand(() -> shooter.presetShoot(Shooter.Positions.SUB_LEFT), shooter))
    );

    joystick.rightBumper()
      .onTrue(new StartEndCommand(
        () -> {intake.setHoming(false);intake.reverseIntake(); System.out.println("right bumper");}, 
        () -> intake.stopIntake(),
        intake
      ).withTimeout(0.1)
      );

    joystick.back().onTrue(new InstantCommand(
      () -> led.toggleHasNote(),
      led
    ));

    // mechstick.povLeft().onTrue(new InstantCommand(
    //   () -> led.setPoliceMode(0),
    //   led
    // ));

    // mechstick.povRight().onTrue(new InstantCommand(
    //   () -> led.setPoliceMode(1),
    //   led
    // ));

    

    joystick.a().onTrue(
      new InstantCommand(() -> intake.reverseIntake(), intake).andThen(new InstantCommand(() -> index.reverseTransfer(), index))
    );

    joystick.x().onTrue(
          new InstantCommand(() -> Shooter.getInstance().setShooterSpeed(1), Shooter.getInstance())
    );
    
    // joystick.y()
    //   .onTrue(new StartEndCommand(
    //     () -> shooter.setShooterSpeed(ampSpeed), 
    //     () -> index.startTransfer(),
    //     index
    //   ).withTimeout(2.5)
    // );

    // joystick.x().onTrue(
    //   new InstantCommand(() -> shooter.setHoming(true))
    // );

    // joystick.y().onTrue(
    //   new InstantCommand(() -> shooter.stow())
    // );

    joystick.y().onTrue(
      new InstantCommand(() -> index.startTransfer(), index)
    );

    joystick.b().onTrue(
      new InstantCommand(() -> index.stopTransfer(), index).andThen(new InstantCommand(() -> shooter.setShooterSpeed(0), shooter))
    );

    

        
    mechstick.back().onTrue(
      new InstantCommand(() -> drivetrain.setDriveState(DriveState.FIELD_CENTRIC_NO_LOCK), drivetrain)
    ).onFalse(
      new InstantCommand(() -> drivetrain.setDriveState(DriveState.FIELD_CENTRIC), drivetrain)
    );
    
    mechstick.start().onTrue(
      drivetrain.resetFieldHeading()
    );

    mechstick.a().onTrue(
      new InstantCommand(
        () -> led.reset(),
        led
      )
    );

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);



    climber.setDefaultCommand(
      new RunCommand(
        () -> climber.move(mechstick.getRightTriggerAxis() - mechstick.getLeftTriggerAxis()), 
        climber)
    );



    // mechstick.y().onTrue(
    //   new InstantCommand(
    //     () -> climber.resetLeftEncoder(), climber
    //   )
    // );
    
    // mechstick.a().onTrue(
    //   new InstantCommand(
    //     () -> climber.resetRightEncoder(), climber
    //   )
    // );
    // m_driverController.x().onTrue(
    //   m_climber.setIdleMode(IdleMode.kCoast)
    // );

    // m_driverController.b().onTrue(
    //   m_climber.setIdleMode(IdleMode.kBrake)
    // );

    joystick.povLeft().onTrue(
      new InstantCommand(
        () -> Shooter.getInstance().setTarget(
          SmartDashboard.getNumber("shoot angle", kAngle)
          ), Shooter.getInstance()
      )
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
  }

  public RobotContainer() {
    configureDashboard();
    configureBindings();    
    configureSelector();
    SmartDashboard.putNumber("kF", 0.025);
    SmartDashboard.putNumber("kP", -5.0); // -1.1
    SmartDashboard.putNumber("encoder target", 0.7);
    SmartDashboard.putNumber("shooter target", 45);
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
      if (!Intake.getInstance().getHoming()){

          angle.HeadingController.setP(SmartDashboard.getNumber("heading p", 4.25));
          angle.HeadingController.setD(SmartDashboard.getNumber("heading d", 0.2));
        } else {
          angle.HeadingController.setP(SmartDashboard.getNumber("homing p", 12));
          angle.HeadingController.setD(SmartDashboard.getNumber("homing d", 0.01));
        
      }

    SmartDashboard.putNumber("current p", angle.HeadingController.getP());
    SmartDashboard.putNumber("current d", angle.HeadingController.getD());

    this.MaxAngularRate = SmartDashboard.getNumber("max turn", CommandSwerveDrivetrain.kTurnSpeed) * Math.PI;
    this.MaxSpeed = SmartDashboard.getNumber("max speed", CommandSwerveDrivetrain.kDriveSpeed);

  }

  public void configureSelector(){
    m_chooser.setDefaultOption("straight", drivetrain.getAuto("StraightLineAuto"));
    // m_chooser.addOption("four note", runAuto);
    m_chooser.addOption("no auto", Commands.print("good luck drivers!"));
    m_chooser.addOption("one note pick up mid", Autos.oneNoteGrabAuto());
    m_chooser.addOption("one note source side", Autos.oneNoteSourceSide());
    m_chooser.addOption("two note", Autos.twoNoteAuto());
    m_chooser.addOption("tow note paths", Autos.twoNotePaths());
    
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

  private double getPredictedHeading(double rate){
    return drivetrain.getCurrentHeadingDegrees() - rate * 1;
  }
}