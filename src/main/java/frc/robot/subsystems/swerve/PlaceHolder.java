package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.pathplanner.lib.auto.AutoBuilder;

public class PlaceHolder extends SwerveDrivetrain{

    public PlaceHolder(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants[] modules) {
        super(driveTrainConstants, modules);
        //TODO Auto-generated constructor stub
        AutoBuilder.configureHolonomic(null, null, null, null, null, null, null);
    }
    
}
