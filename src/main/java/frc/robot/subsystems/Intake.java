package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Intake extends SubsystemBase{

    private final CANSparkMax m_intakeMotor = new CANSparkMax(Constants.Intake.INTAKE_MOTOR_ID, CANSparkMax.MotorType.kBrushless);

    private static Intake instance = null;

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
        this.setSpeed(0.1);//test this when possible
    }

    public void stopIntake() {
        this.setSpeed(0);
    }

    public static Intake getInstance(){
        if (instance == null) instance = new Intake();
        return instance;
    }
}
