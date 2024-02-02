package frc.robot.subsystems.swerve;

/**
 * Interface used by other subsystems to interact and recieve necessary information from Swerve
 */
public interface SwerveIO {
    public double getTargetHeading();
    public double getCurrentHeadingDegrees();
    public void setTargetHeading(double degrees);
}