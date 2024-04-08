package frc.robot.subsystems.localization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerve.generated.TunerConstants;

public class Limelight extends SubsystemBase{
    private NetworkTable limelightTable;
    private double[] pose = new double[6];
    private Pose2d pose2d;
    private boolean tagInVision;
    private boolean snapshot;
    private double distanceFromTag;
    private final double validDistance; // Meters
    private int llNum; // Which number limelight this is

    private static int numOfLimelights;
    private static int activeLimelights;

    private final String name;


    /**
     * @param name the name of the limelight after "limelight-"
     */
    Limelight(String name)
    {
        this(name, 4);
    }

    /**
     * @param name the name of the limelight after "limelight-"
     * @param x the starting x
     * @param y the starting y
     */
    Limelight(String name, double x, double y)
    {
        this(name, 4, x, y);
    }

    /**
     * @param name the name of the limelight after "limelight-"
     * @param distance the number of meters that the limelight can be from a tag before it disables
     */
    Limelight(String name, double distance)
    {
        this(name, distance, 16.579342/2, 5.547868);
    }

    /**
     * @param name the name of the limelight after "limelight-"
     * @param distance the number of meters that the limelight can be from a tag before it disables
     * @param x the starting x
     * @param y the starting y
     */
    Limelight(String name, double distance, double x, double y)
    {
        this.name = "limelight-" + name;
        this.limelightTable = NetworkTableInstance.getDefault().getTable("limelight-" + name);

        this.llNum = numOfLimelights;

        pose[0] = x;
        pose[1] = y;

        this.validDistance = distance;
        numOfLimelights++;
    }

    @Override
    public void periodic() {
        this.tagInVision = this.limelightTable.getEntry("tv").getDouble(0) > 0;

        SmartDashboard.putBoolean(this.name, this.tagInVision);
        
        // Update the location if there is a tag in vision
        if(this.tagInVision)
            this.updateLocation();

        // Mark this limelight as inactive when it is not valid, and active otherwise
        if(this.isValid())
            activeLimelights |= (int)Math.pow(2, this.llNum);
        else
            activeLimelights &= (int)Math.pow(2, numOfLimelights) - (int)Math.pow(2, this.llNum) - 1;
    }

    /**
     * Takes a snapshot of limelight vision.
     * MUST BE CALLED TWICE PER SNAPSHOT
     */
    public void snapshot()
    {
        if(this.snapshot) // Resets the networktables entry for taking a snapshot, allowing another to be taken.
            this.limelightTable.getEntry("snapshot").setNumber(0);
        else // Takes the snapshot
            this.limelightTable.getEntry("snapshot").setNumber(1);
        
        // Invert the value of this.snapshot
        this.snapshot ^= true;
    }

    /**
     * Updates the stored location of the robot using values from the Limelight
     */
    private void updateLocation()
    {
        // Find the robot's current absolute yaw
        double yaw = TunerConstants.DriveTrain.getCurrentHeadingDegrees() + (!DriverStation.getAlliance().isPresent() || DriverStation.getAlliance().get() == Alliance.Blue?0:180);

        // Update LimelightHelpers with the robot's current absolute yaw
        LimelightHelpers.SetRobotOrientation(this.name, yaw, 0, 0, 0, 0, 0);

        
        // MegaTag1 this.pose = this.ll.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
        this.pose = this.limelightTable.getEntry("botpose_orb_wpiblue").getDoubleArray(new double[6]);
        this.pose2d = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name).pose; //new Pose2d(this.pose[0], this.pose[1], new Rotation2d(Math.toRadians(this.pose[5])));
        

        // Get the horizontal distance from the main tag in vision
        // TODO Remove this if we can get the other system to work, cause this one is very not good
        int tid = (int)this.limelightTable.getEntry("tid").getInteger(0);
        if(tid == 0)
            System.out.println("Something went horribly wrong...\nLimelight.updateLocation(), tag id is 0");
        // TODO add the code to actually do this lmao

        double[] cam = this.limelightTable.getEntry("targetpose_cameraspace").getDoubleArray(new double[6]);
        this.distanceFromTag = Math.sqrt(cam[0]*cam[0] + cam[1]*cam[1]);
    }

    /**
     * Gets an array of the values of the pose of the robot as gotten from this limelight
     * @return an array of the values of the pose of the robot
     */
    public double[] getPose()
    {
        return this.pose;
    }

    /**
     * Gets a Pose2d object containing the pose of the robot as gotten from this limelight
     * @return a Pose2d object containing the pose of the robot
     */
    public Pose2d getPose2d()
    {
        return this.pose2d;
    }

    /**
     * Checks if there is a tag in the vision of the Limelight
     * @return a boolean indicating whether there is a tag in the vision of the Limelight or not
     */
    public boolean tiv()
    {
        return this.tagInVision;
    }

    /**
     * Checks if the limelight has a valid reading
     * @return a boolean indicating if the limelight currently has a valid reading or not
     */
    public boolean isValid()
    {
        return this.tagInVision && this.distanceFromTag < this.validDistance;
    }

    /**
     * Gets the number of limelights with valid targets
     * @return the number of active limelights
     */
    public static int getActiveLimelights()
    {
        int health = 0;
        // activeLimelights is interpreted by bit, each bit corresponding to an individual limelight (1 being active)
        // This addsd up each bit to get the total number of active limelights as a regular int
        for(int h = activeLimelights; h > 0; h/=2)
            health += h%2;
        return health;
    }

    /**
     * @return If the Limelight is a 3g or not
     */
    public boolean is3g()
    {
        return !this.name.contains("back");
    }
}