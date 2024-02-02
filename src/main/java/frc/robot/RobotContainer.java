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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.swerve.generated.TunerConstants;
import frc.robot.subsystems.swerve.*;;

public class RobotContainer {
  public static final double kDeadBand = 0.1;
  public static final double kRotationDeadband = 0.05;
  
  private double MaxSpeed = 4.5; 
  private double MaxAngularRate = 0.5 * Math.PI; 

  private final CommandXboxController joystick = new CommandXboxController(0);
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * kDeadBand).withRotationalDeadband(MaxAngularRate * kRotationDeadband)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); 

  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  
  private final SwerveRequest.FieldCentricFacingAngle angle = new SwerveRequest.FieldCentricFacingAngle()
      .withDeadband(MaxSpeed * kDeadBand)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);


  private final Telemetry logger = new Telemetry(MaxSpeed);

  private double targetHeadingD = 0;

  private Command runAuto = drivetrain.getAuto("fournote");

  SendableChooser<String> m_chooser = new SendableChooser<>();

  private void configureBindings() {

    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(
          () -> {
                  if (Math.abs(joystick.getRightX()) > kRotationDeadband || joystick.leftBumper().getAsBoolean()) {
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





    new Trigger(
      () -> Math.abs(joystick.getRightX()) > kRotationDeadband
    ).onFalse(
      new InstantCommand(() -> drivetrain.setTargetHeading(drivetrain.getCurrentHeadingDegrees()), drivetrain)
    );


    angle.HeadingController.setPID(CommandSwerveDrivetrain.kHeadingP, CommandSwerveDrivetrain.kHeadingI, CommandSwerveDrivetrain.kHeadingD);
    angle.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick.b().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

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

  /**
   * Sets up desired values to be displayed to the Smart Dashboard
   */
  public void configureDashboard(){
    SmartDashboard.putNumber("max speed", 1);
    SmartDashboard.putNumber("max turn", 0.5);

    SmartDashboard.putNumber("heading p", 4.25);
    SmartDashboard.putNumber("heading d", 0.2);
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

  /**
   * Updates values needed from the dashboard
   * Called in the main Robot class
   */
  public void updateDashboard(){
    angle.HeadingController.setP(SmartDashboard.getNumber("heading p", 4.25));
    angle.HeadingController.setD(SmartDashboard.getNumber("heading d", 0.2));

    this.MaxAngularRate = SmartDashboard.getNumber("max turn", 0.5) * Math.PI;
    this.MaxSpeed = SmartDashboard.getNumber("max speed", 1);
  }

  /**
   * Re-maps a standard controller joystick input to be more circular and rotationally uniform
   * 
   * @param x
   * @param y
   * @return array of new x and y values
   */
  public static double[] mapJoystick(double x, double y){
    double m_x = x * Math.sqrt(1 - y * y / 2);
    double m_y = y * Math.sqrt(1 - x * x / 2);
    double[] converted = {m_x, m_y};
    return converted;
  }

  private double getPredictedHeading(double rate){
    return drivetrain.getCurrentHeadingDegrees() - rate * 1;
  }
}

