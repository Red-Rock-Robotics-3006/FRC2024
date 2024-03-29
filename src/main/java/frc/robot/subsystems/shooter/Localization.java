package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Localization extends SubsystemBase{
    // private static double robotX = 16.579342/2;
    // private static double robotY = 5.547868;
    // private static double robotYaw = 0;
    private static double[] pose = new double[6];
    private static boolean tagInVision;
    private static boolean snapshot;

    // private static NetworkTable frontLimelight = NetworkTableInstance.getDefault().getTable("limelight-front");
    // private static NetworkTable leftLimelight = NetworkTableInstance.getDefault().getTable("limelight-left");
    // private static NetworkTable rightLimelight = NetworkTableInstance.getDefault().getTable("limelight-right");
    // private static NetworkTable backLimelight = NetworkTableInstance.getDefault().getTable("limelight-back");

    private static Limelight front;
    private static Limelight left;
    private static Limelight right;
    private static Limelight back;

    private static Limelight[] limelights;

    private static int health;

    private static boolean precise;

    public Localization()
    {
        this.setName("Localization");
        this.register();
        
        front = new Limelight("front",8);
        left = new Limelight("left",8);
        right = new Limelight("right",8);
        back = new Limelight("back",4);

        limelights = new Limelight[]{front, left, right, back};
    }

    public void periodic()
    {
        tagInVision = tiv();
        updateLocation();

        
        SmartDashboard.putBoolean("Tag in Vision", tagInVision);
        SmartDashboard.putNumber("Health", health);
        SmartDashboard.putNumber("Bot x", pose[0]);
        SmartDashboard.putNumber("Bot y", pose[1]);
        SmartDashboard.putNumber("Bot z", pose[2]);
        SmartDashboard.putNumber("Bot roll", pose[3]);
        SmartDashboard.putNumber("Bot pitch", pose[4]);
        SmartDashboard.putNumber("Bot yaw", pose[5]);
    }

    private static void updateLocation()
    {
        int denom = 0;
        double[] sum = new double[6];
        for(Limelight l : limelights)
        {
            if(!precise || l.isValid())
            {
                denom++;
                for(int i = 0; i < 6; i++)
                    sum[i] += l.getPose()[i];
            }
        }
        health = denom;

        if((tagInVision || !precise) && denom > 0)
            for(int i = 0; i < 6; i++)
                pose[i] = sum[i] / denom;
    }

    public static Pose2d getPose()
    {
        return new Pose2d(pose[0], pose[1], new Rotation2d(pose[5]));
    }

    private static boolean tiv()
    {
        for(Limelight l : limelights)
            if(l.tiv())
                return true;
        return false;
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
        for(Limelight l : limelights)
            l.snapshot();
    }

    public static Pose2d[] getPose2ds()
    {
        Pose2d[] poses = new Pose2d[limelights.length];
        for(int i = 0; i < poses.length; i++)
            poses[i] = limelights[i].getPose2d();
        return poses;
    }

    public static boolean[] getTags()
    {
        boolean[] tags = new boolean[limelights.length];
        for(int i = 0; i < tags.length; i++)
            tags[i] = limelights[i].tiv();
        return tags;
    }

    private class Limelight extends SubsystemBase{
        private NetworkTable ll;
        private double[] pose = new double[6];
        private Pose2d pose2d;
        private Pose3d pose3d;
        private boolean tagInVision;
        private boolean snapshot;
        private double horizontalDistance;
        private double validDistance = 4; // Meters

        public Limelight(String name)
        {
            this(name, 4);
        }

        public Limelight(String name, double x, double y)
        {
            this(name, 4, x, y);
        }

        public Limelight(String name, double distance)
        {
            this(name, distance, 16.579342/2, 5.547868);
        }

        public Limelight(String name, double distance, double x, double y)
        {
            this.ll = NetworkTableInstance.getDefault().getTable("limelight-" + name);
            pose[0] = x;
            pose[1] = y;
            this.validDistance = distance;
        }
    
    
        @Override
        public void periodic() {
            this.tagInVision = this.ll.getEntry("tv").getDouble(0) > 0;
            if(this.tagInVision)
                this.updateLocation();

            
            // // Set the state of the LEDs to reflect the operating status of the Limelight
            // if(this.isValid())
            //     this.setLED(1); // Disable LEDs when it is valid
            // else if(health == 0)
            //     this.setLED(2); // Flash LEDs when none are valid
            // else
            //     this.setLED(3); // Enable LEDs when it isn't valid
        }


        public void snapshot()
        {
            if(snapshot)
            { // Resets the networktables entry for taking a snapshot, allowing another to be taken.
                this.ll.getEntry("snapshot").setNumber(0);
                snapshot = false;
            }
            else
            { // Takes the snapshot
                snapshot = true;
                this.ll.getEntry("snapshot").setNumber(1);
            }
        }

        private void updateLocation()
        {
            this.pose = this.ll.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
            this.pose2d = new Pose2d(this.pose[0], this.pose[1], new Rotation2d(this.pose[5]));
            this.pose3d = new Pose3d(this.pose[0], this.pose[1], this.pose[2], new Rotation3d(this.pose[3], this.pose[4], this.pose[5]));
            double[] bot = this.ll.getEntry("botpose_targetspace").getDoubleArray(new double[6]);
            this.horizontalDistance = Math.sqrt(bot[0]*bot[0] + bot[1]*bot[1]);
        }

        public double[] getPose()
        {
            return this.pose;
        }

        public Pose2d getPose2d()
        {
            return this.pose2d;
        }

        public Pose3d getPose3d()
        {
            return this.pose3d;
        }

        public boolean tiv()
        {
            return this.tagInVision;
        }

        public boolean isValid()
        {
            return this.tagInVision && this.horizontalDistance < this.validDistance;
        }

        private void setLED(int mode)
        {
            this.ll.getEntry("ledMode").setNumber(mode);
        }
    }
}