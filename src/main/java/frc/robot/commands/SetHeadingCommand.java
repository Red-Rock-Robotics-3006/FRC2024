package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

public class SetHeadingCommand extends Command{
    private CommandSwerveDrivetrain m_drive;

    public SetHeadingCommand(CommandSwerveDrivetrain drive){
        this.m_drive = drive;

        this.addRequirements(drive);
    }

    @Override
    public void execute(){
        double heading = this.m_drive.getState().Pose.getRotation().getDegrees();
        SmartDashboard.putNumber("heading", heading);
    }


}
