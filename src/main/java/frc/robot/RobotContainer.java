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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.swerve.generated.TunerConstants;
import frc.robot.subsystems.swerve.*;;

public class RobotContainer {
  public final double kDeadBand = 0.1;
  public final double kRotationDeadband = 0.05;
  private double MaxSpeed = 1.0; // 6 meters per second desired top speed
  private double MaxAngularRate = 0.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  public final double kTolerance = 0.5;

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * kDeadBand).withRotationalDeadband(MaxAngularRate * kRotationDeadband) // Add a 1% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  
  private final SwerveRequest.FieldCentricFacingAngle angle = new SwerveRequest.FieldCentricFacingAngle()
      .withDeadband(MaxSpeed * kDeadBand)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);


  private final Telemetry logger = new Telemetry(MaxSpeed);

  private double targetHeadingD = 0;

  private Command runAuto = drivetrain.getAuto("StraightLineAuto");

  SendableChooser<String> m_chooser = new SendableChooser<>();

  private void configureBindings() {
    // drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
    //     drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
    //                                                                                        // negative Y (forward)
    //         .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
    //         .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
    //     ));

    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(
          () -> {
                  if (Math.abs(joystick.getRightX()) > kDeadBand) {
                    SmartDashboard.putBoolean("facing angle", false);
                    return drive.withVelocityX(mapJoystick(-joystick.getLeftX(), -joystick.getLeftY())[1] * MaxSpeed)
                                .withVelocityY(mapJoystick(-joystick.getLeftX(), -joystick.getLeftY())[0] * MaxSpeed)
                                .withRotationalRate(-joystick.getRightX() * MaxAngularRate);
                  }
                  else {
                    SmartDashboard.putBoolean("facing angle", true);
                    return angle.withVelocityX(mapJoystick(-joystick.getLeftX(), -joystick.getLeftY())[1] * MaxSpeed)
                                .withVelocityY(mapJoystick(-joystick.getLeftX(), -joystick.getLeftY())[0] * MaxSpeed)
                                .withTargetDirection(new Rotation2d(Math.toRadians(drivetrain.getTargetHeading())));   
                  }    
          }
        )
    );



    // drivetrain.setDefaultCommand(
    //     drivetrain.applyRequest(
    //       () -> {
    //               if (Math.abs(joystick.getRightX()) > kDeadBand) {
    //                 SmartDashboard.putBoolean("facing angle", false);
    //                 return drive.withVelocityX(-joystick.getLeftY() * MaxSpeed)
    //                             .withVelocityY(-joystick.getLeftX() * MaxSpeed)
    //                             .withRotationalRate(-joystick.getRightX() * MaxAngularRate);
    //               }
    //               else {
    //                 SmartDashboard.putBoolean("facing angle", true);
    //                 return angle.withVelocityX(-joystick.getLeftY() * MaxSpeed)
    //                             .withVelocityY(-joystick.getLeftX() * MaxSpeed)
    //                             .withTargetDirection(new Rotation2d(Math.toRadians(targetHeadingD)));   
    //               }    
    //       }
    //     )
    // );

    new Trigger(
      () -> Math.abs(joystick.getRightX()) > kDeadBand
    ).onFalse(
      new InstantCommand(() -> targetHeadingD = this.getDriveHeading(), drivetrain)
    );


    angle.HeadingController.setPID(CommandSwerveDrivetrain.kHeadingP, CommandSwerveDrivetrain.kHeadingI, CommandSwerveDrivetrain.kHeadingD);
    angle.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    // drivetrain.setDefaultCommand(
    //   drivetrain.applyRequest(() ->
    //     angle.withVelocityX(-joystick.getLeftY() * MaxSpeed)
    //          .withVelocityY(-joystick.getLeftX() * MaxSpeed)
    //          .withTargetDirection(new Rotation2d(Math.toRadians(targetHeadingD)))
    //   )
    // );

    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick.b().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // reset the field-centric heading on left bumper press
    // joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    // reset the field centric heading and drivetrain pose heading on y button press
    // joystick.y().onTrue(
    //   drivetrain.runOnce(
    //     () -> drivetrain.seedFieldRelative(
    //       new Pose2d(
    //         drivetrain.getState().Pose.getX(),
    //         drivetrain.getState().Pose.getY(),
    //         new Rotation2d()
    //       )
    //     )
    //   )
    // );

    // joystick.y().onTrue(
    //   new SequentialCommandGroup(
    //     new InstantCommand(() -> drivetrain.seedFieldRelative(
    //       new Pose2d(
    //         drivetrain.getState().Pose.getX(),
    //         drivetrain.getState().Pose.getY(),
    //         new Rotation2d()
    //       )
    //     ), drivetrain
    //   ),
    //   new InstantCommand(() -> targetHeadingD = 0)
    // ));

    joystick.y().onTrue(
      drivetrain.resetHeading()
    );
    
    RobotModeTriggers.teleop().onTrue(
      drivetrain.resetHeading()
    );

    joystick.povLeft().onTrue(
      new InstantCommand(() -> drivetrain.setTargetHeading(90), drivetrain)
    );

    joystick.povDown().onTrue(
      new InstantCommand(() -> drivetrain.setTargetHeading(180), drivetrain)
    );

    joystick.povRight().onTrue(
      new InstantCommand(() -> drivetrain.setTargetHeading(-90), drivetrain)
    );

    joystick.povUp().onTrue(
      new InstantCommand(() -> drivetrain.setTargetHeading(0), drivetrain)
    );

    

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public void configureDashboard(){
    SmartDashboard.putNumber("max speed", 0.5);
    SmartDashboard.putNumber("p value", 4.25);
    SmartDashboard.putNumber("d value", 0.2);


  }

  public RobotContainer() {
    configureDashboard();
    configureBindings();
  }

  public Command getAutonomousCommand() {
    // return Commands.print("No autonomous command configured");  
    return runAuto;
  }

  public double getDriveHeading(){
    return this.drivetrain.getState().Pose.getRotation().getDegrees();
  }

  public double getTargetHeading(){

    return this.targetHeadingD;
  }

  public void updateDashboard(){
    angle.HeadingController.setP(SmartDashboard.getNumber("p value", 1));
    angle.HeadingController.setD(SmartDashboard.getNumber("d value", 0.2));

    this.MaxAngularRate = SmartDashboard.getNumber("max speed", 0.5) * Math.PI;
  }

  public static double[] mapJoystick(double x, double y){
    double m_x = x * Math.sqrt(1 - y * y / 2);
    double m_y = y * Math.sqrt(1 - x * x / 2);
    double[] converted = {m_x, m_y};
    return converted;
  }
}

