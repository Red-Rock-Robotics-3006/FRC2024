package frc.robot.subsystems.localization;

import java.util.Arrays;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Localization extends SubsystemBase{
    private static Localization instance = new Localization();
    private static double[] pose = new double[6];
    private static Pose2d pose2d = new Pose2d();
    private static boolean tagInVision;

    private static Limelight front;
    private static Limelight left;
    private static Limelight right;
    private static Limelight back;

    private static Limelight[] limelights;

    // private static boolean precise;

    public Localization()
    {
        super("Localization");
        
        front = new Limelight("front",8);
        left = new Limelight("left",8);
        right = new Limelight("right",8);
        // back = new Limelight("back",4);

        limelights = new Limelight[]{front, left, right}; //TODO Add back the other limelights?

        /* Remove any unwanted tags from the array
         * Guide:
         * 1: Blue Source Right
         * 2: Blue Source Left
         * 3: Red Speaker Right
         * 4: Red Speaker Left
         * 5: Red Amp
         * 6: Blue Amp
         * 7: Blue Speaker Right
         * 8: Blue Speaker Left
         * 9: Red Source Right
         * 10: Red Source Left
         * 11: Red Stage Source
         * 12: Red Stage Amp
         * 13: Red Stage Center
         * 14: Blue Stage Center
         * 15: Blue Stage Amp
         * 16: Blue Stage Source
         */
        int[] validIDs = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16};
        LimelightHelpers.SetFiducialIDFiltersOverride("limelight-front", validIDs);
        LimelightHelpers.SetFiducialIDFiltersOverride("limelight-left", validIDs);
        LimelightHelpers.SetFiducialIDFiltersOverride("limelight-right", validIDs);
        LimelightHelpers.SetFiducialIDFiltersOverride("limelight-back", validIDs);
    }

    @Override
    public void periodic()
    {
        tagInVision = vtiv();

        // Only update the location if at least 1 Limelight is active
        if(tagInVision)
            updateLocation();

        
        SmartDashboard.putBoolean("Tag in Vision", tagInVision);
        SmartDashboard.putNumber("Number of Active Limelights", Limelight.getActiveLimelights());
        SmartDashboard.putNumber("Bot x", pose2d.getX());
        SmartDashboard.putNumber("Bot y", pose2d.getY());
        SmartDashboard.putNumber("Bot z", pose[2]);
        SmartDashboard.putNumber("Bot roll", pose[3]);
        SmartDashboard.putNumber("Bot pitch", pose[4]);
        SmartDashboard.putNumber("Bot yaw", pose2d.getRotation().getDegrees());
    }

    /**
     * Updates the stored location of the robot using data from the Limelights
     */
    private static void updateLocation()
    {
        // Store the average pose
        double[] avgPose = new double[6];
        // Iterate over each part of the pose (x,y,z,roll,pitch,yaw)
        for(int i = 0; i < avgPose.length; i++)
        {
            double sum = 0;
            // Iterate over each limelight
            for(Limelight l : limelights)
                // If the limelight is active, add its values to the sum
                if(l.isValid())
                    sum += l.getPose()[i];
            avgPose[i] = sum/Limelight.getActiveLimelights();
        }
        pose = avgPose;


        // Update the average Pose2d
        double[] sums = new double[3];
        for(Limelight l : limelights)
        {
            if(!l.isValid())
                continue;
            Pose2d p = l.getPose2d();
            sums[0] += p.getX();
            sums[1] += p.getY();
            sums[2] += p.getRotation().getDegrees();
        }
        pose2d = new Pose2d(sums[0], sums[1], new Rotation2d(Math.toRadians(sums[2])));

        /* Old code
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

        if((tagInVision || !precise) && denom > 0)
            for(int i = 0; i < 6; i++)
                pose[i] = sum[i] / denom;*/
    }

    /**
     * @return If there is a valid tag in vision
     */
    public static boolean vtiv()
    {
        return Limelight.getActiveLimelights() > 0;
    }

    /**
     * @return If there is a tag in vision
     */
    public static boolean tiv()
    {
        for(Limelight l : limelights)
            if(l.tiv())
                return true;
        return false;
    }

    /**
     * Calculates an average of the poses from each active limelight.
     * @return The last known Pose of the robot
     */
    public static Pose2d getPose()
    {
        return pose2d;
    }

    /**
     * @return If at least 1 Limelight can see an AprilTag
     */
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

    /**
     * @return An array of all Pose2ds from the Limelights
     */
    public static Pose2d[] getPose2ds()
    {
        Pose2d[] poses = new Pose2d[limelights.length];
        Arrays.setAll(poses, n->limelights[n].getPose2d());
        // for(int i = 0; i < poses.length; i++)
        //     poses[i] = limelights[i].getPose2d();
        return poses;
    }

    /**
     * @return An array of if each Limelight can see an AprilTag
     */
    public static boolean[] getTags()
    {
        boolean[] tags = new boolean[limelights.length];
        for(int i = 0; i < tags.length; i++) {
            tags[i] = limelights[i].tiv();
        }
        return tags;
    }

    /**
     * @return An array of all Limelight objects being used by <code>Localization</code>
     */
    public static Limelight[] getLimeLights(){
        return limelights;
    }
}