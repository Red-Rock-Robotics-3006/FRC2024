package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveIO;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain.DriveState;
import frc.robot.subsystems.swerve.generated.TunerConstants;


public class Intake extends SubsystemBase{

    //private final CANSparkMax m_intakeMotor = new CANSparkMax(Constants.Intake.INTAKE_MOTOR_ID, CANSparkMax.MotorType.kBrushless);

    private static Intake instance = null;
    private static boolean homing = false;
    private int periodicControl = 0;
    private double boundingBoxOffsetX = 1.0;
    private double boundingBoxOffsetY = 1.0;
    private double limelightPoseOffset = 13.653;
    private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    private SwerveIO swerve = TunerConstants.DriveTrain;//TODO filler for no ugly red

    private double x, y, z, a, b, c;

    private Intake() {
        this.setName("Intake");
        this.register();

        // this.m_intakeMotor.restoreFactoryDefaults();
        // this.m_intakeMotor.setInverted(false);
        // this.m_intakeMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    /**
     * Sets speed of intake motor to specified parameter
     * @param speed user specified parameter
     */
    // public void setSpeed(double speed) {
    //     this.m_intakeMotor.set(speed);
    // }

    /**
     * Sets motor speed at a certain speed
    //  */
    // public void startIntake() {
    //     this.setSpeed(0.1);//test this when possible. will likely be set high to something like 0.97 or 1
    // }
    
    // /**
    //  * Stops the motor
    //  */
    // public void stopIntake() {
    //     this.setSpeed(0);
    // }

    /**
     * Detects whether or not a note is within the Limelight's POV
     * @return whether or note a note is detected
     */
    public boolean noteDetected() {
        return table.getEntry("tv").getDouble(0) > 0;
    }

    /**
     * Gives the horizontal angular displacement of a detected note in degrees
     * @return horizontal displacement in degrees
     */
    public double getNoteDegreeX() {
        return table.getEntry("tx").getDouble(0) + this.boundingBoxOffsetX;
    }

    /**
     * Gives the vertical angular displacement of a detected note in degrees
     * @return vertical displacement in degrees
     */
    public double getNoteDegreeY() {
        return table.getEntry("ty").getDouble(0) + this.boundingBoxOffsetY;
    }

    public double getNoteBoundingBoxWidth() {
        return table.getEntry("thor").getDouble(0);
    }

    public double calculateX(double boundingBoxWidth) {
        return 600 / boundingBoxWidth; //TODO implement this method
    }

    public double calculateZ(double x, double y, double a) {
        return Math.sqrt(x * x + y * y - 2 * x * y * Math.cos(this.degreesToRadians(a)));
    }

    public double calculateHeading(double x, double y, double z) {
        return this.swerve.getCurrentHeadingDegrees() - this.radiansToDegrees(Math.acos((x * x - y * y - z * z) / (-2 * y * z))) * Math.signum(this.getNoteDegreeX());
    }

    public double degreesToRadians(double degrees) {
        return degrees * (Math.PI / 180);
    }

    public double radiansToDegrees(double degrees) {
        return degrees * (180 / Math.PI);
    }

    public void setHoming(boolean b) {
        Intake.homing = b;
    }

    public boolean getHoming() {
        return Intake.homing;
    }

    public void periodic() {
        this.c = this.getNoteDegreeX();
        this.b = 180 - this.c;
        this.x = this.calculateX(this.getNoteBoundingBoxWidth());
        this.y = this.limelightPoseOffset;
        this.z = this.calculateZ(x, y, b);
        this.a = this.calculateHeading(x, y, z);
        
        if (homing) {
            if (this.noteDetected()) {
                //this.startIntake();
                this.swerve.setTargetHeading(this.a);
                if (Math.abs(this.getNoteDegreeX()) < 10 && this.x < 24) this.swerve.setDriveState(DriveState.ROBOT_CENTRIC);
                else this.swerve.setDriveState(DriveState.FIELD_CENTRIC);
            }
        }
        else {
            //this.stopIntake();
            this.swerve.setDriveState(DriveState.FIELD_CENTRIC);
        }
    }

    public static Intake getInstance(){
        if (instance == null) instance = new Intake();
        return instance;
    }
}