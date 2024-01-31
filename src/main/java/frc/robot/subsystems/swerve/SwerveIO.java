package frc.robot.subsystems.swerve;

public interface SwerveIO {
    public double getTargetHeading();
    public double getCurrentHeadingDegrees();
    public void setTargetHeading(double degrees);
}
