package frc.robot.subsystems.swerve;

// import frc.robot.subsystems.swerve.CommandSwerveDrivetrain.DriveState;

/**
 * Interface used by other subsystems to interact and recieve necessary information from Swerve
 */
public interface SwerveIO {

    
    public double getTargetHeading();
    // public double getCurrentHeadingDegrees();
    public void setTargetHeading(double degrees);
    public boolean isStill();
    // public void setDriveState(DriveState state);
}