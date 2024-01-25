package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AmpBar extends SubsystemBase{
    private static AmpBar instance;

    private CANSparkMax ampBarMotor = new CANSparkMax(Constants.AmpBar.AMP_BAR_MOTOR_ID, MotorType.kBrushless); // REPLACE FILLER MOTOR ID
    
    private double targetAngle = 0;

    private double kP = 0.00000000001;
    private double kI = 0;
    private double kD = 0;
    private double kF = 0.00000000001;

    private double topAngle = 120;
    private double bottomAngle = 0;

    private double currentAngle = 0;

    private PIDController controller = new PIDController(kP, kI, kD);
    private RelativeEncoder encoder = ampBarMotor.getEncoder();

    private AmpBar() {
        this.setName("Amp Bar");
        this.register();

        this.ampBarMotor.restoreFactoryDefaults();

        this.ampBarMotor.setInverted(false);

        this.ampBarMotor.setIdleMode(IdleMode.kBrake);
    }

    /**
     * Stops amp bar in place
     */
    public void freeze() {
        this.ampBarMotor.set(0);
    }

    /**
     * Get angle of motor currently
     * @return current angle of motor
     */
    public double getAngle() {
        double angle;
        double currentTick = encoder.getPosition();
        angle = 6.0645 * (currentTick + 95.63);
        return angle;
    }

    /**
     * Converts angle to ticks
     * @param angle angle to convert to ticks
     * @return ticks from given angle
     */
    public double toTicks(double angle) {
        double ticks = (angle / 6.0645) - 95.63;
        return ticks;
    }

    /**
     * Sets target angle to stow angle
     */
    public void goBottom() {
        this.targetAngle = this.bottomAngle;
    }

    /**
     * Sets target angle to active angle
     */
    public void goTop() {
        this.targetAngle = this.topAngle;
    }

    /**
     * Returns an instance of AmpBar if one exists, otherwise creates one
     * @return AmpBar instance
     */
    public static AmpBar getInstance() {
        if (AmpBar.instance == null) {
            AmpBar.instance = new AmpBar();
        }

        return AmpBar.instance;
    }
    
    @Override
    public void periodic() {
        this.controller.setPID(this.kP, this.kI, this.kD);
        this.currentAngle = this.getAngle();
        double feedfoward = kF * Math.cos(Math.toRadians(currentAngle));
        this.ampBarMotor.set(this.controller.calculate(this.encoder.getPosition(), this.toTicks(this.targetAngle)) + feedfoward);
    }
}
