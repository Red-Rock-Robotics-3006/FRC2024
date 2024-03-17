package frc.robot.subsystems;



import com.ctre.phoenix6.hardware.Pigeon2;


import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Localization extends SubsystemBase {
    // private static Pigeon2 gyro = new Pigeon2(0);

    private static double robotX;
    private static double robotY;
    private static double robotZ;
    private static double robotRoll;
    private static double robotPitch;
    private static double robotYaw;

    private static NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");

    public void periodic()
    {
        updateLocation();
    }

    public static boolean tagInVision()
    {
        return limelight.getEntry("tv").getDouble(0) > 0;
    }


    public static void updateLocation()
    {
        if(tagInVision())
        {
            double[] pose = limelight.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
            robotX = pose[0];
            robotY = pose[1];
            robotZ = pose[2];
            robotRoll = pose[3];
            robotPitch = pose[4];
            robotYaw = pose[5];
        }
    }

    // public static void moveLocation()
    // {
    //     if(!tagInVision())
    //     {
    //         gyro.getAccelerationX().
    //     }
    // }

    public static double[] getLocation()
    {
        return new double[]{robotX, robotY, robotZ, robotRoll, robotPitch, robotYaw};
    }
}
