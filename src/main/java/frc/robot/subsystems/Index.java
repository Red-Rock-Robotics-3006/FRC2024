package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Index extends SubsystemBase{

    private final CANSparkMax m_frontMotor = new CANSparkMax(Constants.Index.TOP_MOTOR_ID, CANSparkMax.MotorType.kBrushless);
    private final CANSparkMax m_backMotor = new CANSparkMax(Constants.Index.BOTTOM_MOTOR_ID, CANSparkMax.MotorType.kBrushless);
    private final DigitalInput beamBrake = new DigitalInput(Constants.Index.SWITCH_CHANNEL_ID);

    private static Index instance = null;

    private Index() {
        this.setName("Index");
        this.register();

        this.m_frontMotor.restoreFactoryDefaults();
        this.m_frontMotor.setInverted(false);
        this.m_frontMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        this.m_backMotor.restoreFactoryDefaults();
        this.m_backMotor.setInverted(false);
        this.m_backMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    public void setSpeed(double speed) {
        this.m_frontMotor.set(speed);
        this.m_backMotor.set(speed);
    }

    public void startTransfer() {
        this.setSpeed(0.1);//test this when possible
    }

    public void stopTransfer() {
        this.setSpeed(0);
    }

    public boolean hasNote() {
        if (this.beamBrake.get()) return true;
        return false;
    }

    /**
     * Singleton architecture which returns the singular instance of Index
     * @return the instance (which is instantiated when first called)
     */
    public static Index getInstance(){
        if (instance == null) instance = new Index();
        return instance;
    }
}