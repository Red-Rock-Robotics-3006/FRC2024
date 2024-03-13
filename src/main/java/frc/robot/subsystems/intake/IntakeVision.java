package frc.robot.subsystems.intake;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveIO;
import frc.robot.subsystems.swerve.generated.TunerConstants;

public class IntakeVision extends SubsystemBase {

    private static IntakeVision instance = null;

    private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-intake");
    private SwerveIO swerve = TunerConstants.DriveTrain;

    private boolean homing = false;
    private double boundingBoxOffsetX = 1.0;
    private double boundingBoxOffsetY = 1.0;
    private double limelightPoseOffset = 13.653;
    private double x, y, z, a, b, c;

    private IntakeVision() {
        this.setName("Intake Vision");
        this.register();
    }

    public boolean noteDetected() {
        return this.table.getEntry("tv").getDouble(0) > 0;
    }

    public double getNoteDegreeX() {
        return this.table.getEntry("tx").getDouble(0) + this.boundingBoxOffsetX;
    }

    public double getNoteDegreeY() {
        return this.table.getEntry("ty").getDouble(0) + this.boundingBoxOffsetY;
    }

    public double getNoteBoundingBoxWidth() {
        return this.table.getEntry("thor").getDouble(0);
    }

    public double calculateX(double boundingBoxWidth) {
        return 600 / boundingBoxWidth;
    }

    public double calculateZ(double x, double y, double a) {
        return Math.sqrt(x * x + y * y - 2 * x * y * Math.cos(Math.toRadians(a)));
    }

    public double calculateHeading(double x, double y, double z) {
        return this.swerve.getCurrentHeadingDegrees() - Math.toDegrees(Math.acos((x * x - y * y - z * z) / (-2 * y * z))) * Math.signum(this.getNoteDegreeX());
    }

    public void setHoming(boolean b) {
        this.homing = b;
    }

    public boolean getHoming() {
        return this.homing;
    }

    public void toggleHoming() {
        if (this.getHoming()) this.setHoming(false);
        else this.setHoming(true);
    }
    
    public void periodic() {
        if (this.homing) {
            this.c = this.getNoteDegreeX();
            this.b = 180 - this.c;
            this.x = this.calculateX(this.getNoteBoundingBoxWidth());
            this.y = this.limelightPoseOffset;
            this.z = this.calculateZ(x, y, b);
            this.a = this.calculateHeading(x, y, z);
            if (Constants.Settings.INTAKE_HOMING_ENABLED) this.swerve.setTargetHeading(this.a);
            SmartDashboard.putNumber("a value", this.a);
        }
    }

    public static IntakeVision getInstance() {
        if (instance == null) instance = new IntakeVision();
        return instance;
    }
}
