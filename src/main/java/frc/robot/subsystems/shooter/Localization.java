package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Localization extends SubsystemBase{
    private static double robotX = 16.579342/2;
    private static double robotY = 5.547868;
    private static double robotYaw = 0;
    private static boolean tagInVision;
    private static boolean snapshot;

    private static NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight-intake");

    public Localization()
    {
        this.setName("Localization");
        this.register();
    }

    public void periodic()
    {
        this.updateLocation();

        SmartDashboard.putBoolean("Tag in Vision", tagInVision);
        SmartDashboard.putNumber("Bot x", robotX);
        SmartDashboard.putNumber("Bot y", robotY);
        SmartDashboard.putNumber("Bot yaw", robotYaw);
    }

    private void updateLocation()
    {
        tagInVision = limelight.getEntry("tv").getDouble(0) > 0;

        if(tagInVision)
        {
            double[] pose = limelight.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
            robotX = pose[0];
            robotY = pose[1];
            robotYaw = pose[5];
        }
    }

    public static Pose2d getPose()
    {
        return new Pose2d(robotX, robotY, new Rotation2d(robotYaw));
    }

    public static boolean tagInVision()
    {
        return tagInVision;
    }

    /**
     * Takes a snapshot of limelight vision.
     * MUST BE CALLED TWICE PER SNAPSHOT
     */
    public static void takeSnapshot()
    {
        if(snapshot)
        { // Resets the networktables entry for taking a snapshot, allowing another to be taken.
            limelight.getEntry("snapshot").setNumber(0);
            snapshot = false;
        }
        else
        { // Takes the snapshot
            snapshot = true;
            limelight.getEntry("snapshot").setNumber(1);
        }
    }
}
