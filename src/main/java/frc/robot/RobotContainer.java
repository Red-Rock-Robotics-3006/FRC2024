// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.*;

public class RobotContainer {

  private final CommandXboxController m_driveControl = new CommandXboxController(1);//FILLER
  private final CommandXboxController m_mechControl = new CommandXboxController(2);//FILLER

  Intake intake = Intake.getInstance();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    m_mechControl.a()
    .onTrue(new InstantCommand(
        () -> intake.startIntake(),
        intake));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}