package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkFlex;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase{

    private static Intake instance = null;

    private final CANSparkFlex m_intakeMotor = new CANSparkFlex(Constants.Intake.INTAKE_MOTOR_ID, CANSparkFlex.MotorType.kBrushless);

    private double kIntakeSpeed = 1;
    public static final double kRollForwardTime = 0.1;

    private Intake() {
        super("Intake");

        this.m_intakeMotor.restoreFactoryDefaults();
        this.m_intakeMotor.setInverted(false);
        this.m_intakeMotor.setIdleMode(CANSparkFlex.IdleMode.kBrake);
    }

    public void burnFlash() {
        this.m_intakeMotor.burnFlash();
    }

    public void setSpeed(double speed) {
        this.m_intakeMotor.set(speed);
    }

    public void startIntake() {
        this.setSpeed(this.kIntakeSpeed);
    }

    public void reverseIntake() {
        this.setSpeed(-0.3);
    }
    
    public void stopIntake() {
        this.setSpeed(0);
    }

    public static Intake getInstance(){
        if (instance == null) instance = new Intake();
        return instance;
    }
}