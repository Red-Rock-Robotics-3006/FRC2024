package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveIO;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain.DriveState;
import frc.robot.subsystems.swerve.generated.TunerConstants;


public class Intake extends SubsystemBase{

    private static Intake instance = null;

    private final CANSparkFlex m_intakeMotor = new CANSparkFlex(Constants.Intake.INTAKE_MOTOR_ID, CANSparkMax.MotorType.kBrushless);

    private Index index = Index.getInstance();
    private SwerveIO swerve = TunerConstants.DriveTrain;
    private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-intake");

    private boolean homing = false;
    private double boundingBoxOffsetX = 1.0;
    private double boundingBoxOffsetY = 1.0;
    private double limelightPoseOffset = 13.653;
    private double kIntakeSpeed = 0.55;
    private double x, y, z, a, b, c;

    private Intake() {
        this.setName("Intake");
        this.register();

        this.m_intakeMotor.restoreFactoryDefaults();
        this.m_intakeMotor.setInverted(false);
        this.m_intakeMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        this.m_intakeMotor.burnFlash();
    }

    /**
     * Sets speed of intake motor to specified parameter
     * @param speed user specified parameter
     */
    public void setSpeed(double speed) {
        this.m_intakeMotor.set(speed);
        // System.out.println("set speed");
    }

    /**
     * Sets motor speed at a certain speed
     */
    public void startIntake() {
        this.setSpeed(this.kIntakeSpeed);
        index.startTransfer();
        // System.out.println("start intake");
    }

    /**
     * Spins intake backwards in the case of a note being caught
     */
    public void reverseIntake() {
        // this.setHoming(false);
        this.setSpeed(-0.2);
        index.reverseTransfer();
    }
    
    /**
     * Stops the motor
     */
    public void stopIntake() {
        this.setSpeed(0);
        index.stopTransfer();
    }

    /**
     * Detects whether or not a note is within the Limelight's POV
     * @return whether or note a note is detected
     */
    public boolean noteDetected() {
        return this.table.getEntry("tv").getDouble(0) > 0;
    }

    /**
     * Gives the horizontal angular displacement of a detected note in degrees
     * @return horizontal displacement in degrees
     */
    public double getNoteDegreeX() {
        return this.table.getEntry("tx").getDouble(0) + this.boundingBoxOffsetX;
    }

    /**
     * Gives the vertical angular displacement of a detected note in degrees
     * @return vertical displacement in degrees
     */
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
        index.setTransferring(b);
    }

    public boolean getHoming() {
        return this.homing;
    }

    public void toggleHoming() {
        if (this.getHoming()) this.setHoming(false);
        else {
            this.setHoming(true);
            index.setTransferring(true);
        }
    }
    
    public void periodic() {
        // this.c = this.getNoteDegreeX();
        // this.b = 180 - this.c;
        // this.x = this.calculateX(this.getNoteBoundingBoxWidth());
        // this.y = this.limelightPoseOffset;
        // this.z = this.calculateZ(x, y, b);
        // this.a = this.calculateHeading(x, y, z);
        
        if (this.homing) {
            this.startIntake();
            if (Constants.Settings.INTAKE_HOMING_ENABLED) this.swerve.setTargetHeading(this.a);
            if (index.getTransferring()) this.index.startTransfer();
            // SmartDashboard.putNumber("a value", this.a);
            // SmartDashboard.putNumber("note degree x", this.c);
            // if (Math.abs(this.getNoteDegreeX()) < 10 && this.x < 24) this.swerve.setDriveState(DriveState.ROBOT_CENTRIC);//TODO may or may not use this
            // else this.swerve.setDriveState(DriveState.FIELD_CENTRIC);
        }
        // else if (!this.homing) {
        //     this.stopIntake();
        //     // this.swerve.setDriveState(DriveState.FIELD_CENTRIC);
        //     this.index.stopTransfer();
        // }
    }

    /**
     * Singleton architecture which returns the singular instance of Intake
     * @return the instance (which is instantiated when first called)
     */
    public static Intake getInstance(){
        if (instance == null) instance = new Intake();
        return instance;
    }
}