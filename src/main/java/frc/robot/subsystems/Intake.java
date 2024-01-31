package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Intake extends SubsystemBase{

    private final CANSparkMax m_intakeMotor = new CANSparkMax(Constants.Intake.INTAKE_MOTOR_ID, CANSparkMax.MotorType.kBrushless);

    private static Intake instance = null;
    private static boolean homing = false;
    private int periodicControl = 0;
    private double boundingBoxOffsetX = 1.0;
    private double boundingBoxOffsetY = 1.0;
    private double limelightPoseOffset = 1.0;
    private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    private Swerve swerve = new Swerve();//TODO filler for no ugly red

    private double x, y, z, a, b;

    private Intake() {
        this.setName("Intake");
        this.register();

        this.m_intakeMotor.restoreFactoryDefaults();
        this.m_intakeMotor.setInverted(false);
        this.m_intakeMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    public void setSpeed(double speed) {
        this.m_intakeMotor.set(speed);
    }

    public void startIntake() {
        this.setSpeed(0.1);//test this when possible. will likely be set high to something like 0.97 or 1
    }

    public void stopIntake() {
        this.setSpeed(0);
    }

    public double getNoteDegreeX() {
        return table.getEntry("tx").getDouble(0) + this.boundingBoxOffsetX;
    }

    public double getNoteDegreeY() {
        return table.getEntry("ty").getDouble(0) + this.boundingBoxOffsetY;
    }

    public double getNoteBoundingBoxWidth() {
        return table.getEntry("thor").getDouble(0);
    }

    public double calculateX(double boundingBoxWidth) {
        return 0; //TODO implement this method
    }

    public double calculateZ(double x, double y, double a) {
        return Math.sqrt(x * x + y * y - 2 * x * y * Math.cos(this.degreesToRadians(a)));
    }

    public double calculateHeading(double x, double y, double z) {
        return this.swerve.getHeading() + this.radiansToDegrees(Math.acos((x * x - y * y - z * z) / (-1 * 2 * y * z))) * Math.signum(this.getNoteDegreeX());
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
        periodicControl++; //will probably remove this since it will lag intake
        if (periodicControl % 500 == 0) { //this does things every second which is probably slow but whatever will change it anyways
            this.periodicControl = 0;
            this.b = 180 - this.getNoteDegreeX();
            this.x = this.calculateX(this.getNoteBoundingBoxWidth());
            this.y = this.limelightPoseOffset;
            this.z = this.calculateZ(x, y, b);
            this.a = this.calculateHeading(x, y, z);

            if (homing) {
                this.startIntake();
                this.swerve.setDriveMode("robot centric");
                this.swerve.setHeading(this.a);
            }
            else {
                this.stopIntake();
                this.swerve.setDriveMode("field centric");
            }
        }
    }

    public static Intake getInstance(){
        if (instance == null) instance = new Intake();
        return instance;
    }
}