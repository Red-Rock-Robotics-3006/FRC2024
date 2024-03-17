// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final Shooter m_shooter = Shooter.getInstance();

  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  public RobotContainer() {
    configureBindings();
    SmartDashboard.putNumber("kF", 0.046);
    SmartDashboard.putNumber("kP", 0);
    SmartDashboard.putNumber("encoder target", 0.7);
  }

  private void configureBindings() {
    // new Trigger(m_exampleSubsystem::exampleCondition)
    //     .onTrue(new ExampleCommand(m_exampleSubsystem));

    m_driverController.b().onTrue(new InstantCommand(() -> {m_shooter.presetShoot(Shooter.Positions.SUB_CENTER);}));
    m_driverController.a().onTrue(new InstantCommand(() -> {
      m_shooter.setAngleSpeed(0.1);
    })).onFalse(new InstantCommand(() -> {
      m_shooter.setAngleSpeed(0);
    }));  
  }

  public Command getAutonomousCommand() {
    return null;
  }
  

  /**
   * Updates values needed from the dashboard
   * Called in the main Robot class
   */
    public void updateDashboard(){



    //   if (!Intake.getInstance().getHoming()){

    //       angle.HeadingController.setP(SmartDashboard.getNumber("heading p", 4.25));
    //       angle.HeadingController.setD(SmartDashboard.getNumber("heading d", 0.2));
    //     } else {
    //       angle.HeadingController.setP(SmartDashboard.getNumber("homing p", 12));
    //       angle.HeadingController.setD(SmartDashboard.getNumber("homing d", 0.01));
        
    //   }

    // SmartDashboard.putNumber("current p", angle.HeadingController.getP());
    // SmartDashboard.putNumber("current d", angle.HeadingController.getD());

    // this.MaxAngularRate = SmartDashboard.getNumber("max turn", 0.5) * Math.PI;
    // this.MaxSpeed = SmartDashboard.getNumber("max speed", 1);

    // Intake.kIntakeSpeed = SmartDashboard.getNumber("intake speed", 0.7);
  }
}
