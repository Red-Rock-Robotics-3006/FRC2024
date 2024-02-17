package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//TODO random filler deelte this later
public class Shooter extends SubsystemBase{
    private static Shooter instance = null;

    private boolean isActive = false;

    public static double kShooterSpeed = 0.5;

    private final CANSparkFlex m_leftMotor = new CANSparkFlex(59, CANSparkMax.MotorType.kBrushless); //59
    private final CANSparkFlex m_rightMotor = new CANSparkFlex(42, CANSparkMax.MotorType.kBrushless); //42

    private Shooter(){
        this.setName("Shooter");
        this.register();

        this.m_leftMotor.restoreFactoryDefaults();
        this.m_leftMotor.setInverted(false);
        this.m_leftMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);

        this.m_rightMotor.restoreFactoryDefaults();
        this.m_rightMotor.setInverted(true);
        this.m_rightMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    }

    public void setSpeed(double speed){
        this.m_leftMotor.set(speed);
        this.m_rightMotor.set(speed);
    }

    public void startShooter(){
        this.setSpeed(kShooterSpeed);
        this.isActive = true;
    }

    public void stopShooter(){
        this.setSpeed(0);
        this.isActive = false;
    }

    public void toggleShooter(){
        if (isActive) stopShooter();
        else startShooter();
    }

    public static Shooter getInstance() {
        if (instance == null) instance = new Shooter();
        return instance;
    }

    public boolean getHoming() {
        return false;
    }
}
