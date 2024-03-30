package frc.robot.subsystems.index;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Index extends SubsystemBase{
    
    private static Index instance = null;

    private final CANSparkFlex m_indexMotor = new CANSparkFlex(Constants.Index.INDEX_MOTOR_ID, CANSparkFlex.MotorType.kBrushless);
    private final CANSparkMax m_rollerMotor = new CANSparkMax(Constants.Index.ROLLER_MOTOR_ID, CANSparkMax.MotorType.kBrushless);
    
    public static final double kReverseTime = 0.1;

    public static final double kIndexSpeed = 0.34;
    public static final double kIndexShootSpeed = 0.12;
    public static final double kRollerSpeed = 0.95;

    private boolean isTransferring = false;

    private Index() {
        this.setName("Index");
        this.register();

        this.m_indexMotor.restoreFactoryDefaults();
        this.m_indexMotor.setInverted(true);
        this.m_indexMotor.setIdleMode(CANSparkFlex.IdleMode.kBrake);

        this.m_rollerMotor.restoreFactoryDefaults();
        this.m_rollerMotor.setInverted(true);
        this.m_rollerMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        SmartDashboard.putNumber("index reverse time", kReverseTime);

        SmartDashboard.putNumber("index speed", kIndexSpeed);
        SmartDashboard.putNumber("roller speed", kRollerSpeed);
        SmartDashboard.putNumber("index shoot speed", kIndexShootSpeed);
    }

    public void setTransferring(boolean b) {
        this.isTransferring = b;
    }

    public boolean getTransferring() {
        return this.isTransferring;
    }

    public void setSpeed(double speed) {
        this.m_indexMotor.set(speed);
        this.m_rollerMotor.set(speed);
    }

    public void startTransfer() {
        this.m_rollerMotor.set(SmartDashboard.getNumber("roller speed", kRollerSpeed));
        this.m_indexMotor.set(SmartDashboard.getNumber("index speed", kIndexSpeed));
    }

    public void shootTransfer(){
        this.m_rollerMotor.set(SmartDashboard.getNumber("roller speed", kRollerSpeed));
        this.m_indexMotor.set(SmartDashboard.getNumber("index shoot speed", kIndexShootSpeed));
    }

    public void stopTransfer() {
        this.isTransferring = false;
        this.setSpeed(0);
    }

    public void reverseTransfer() {
        this.setSpeed(-0.5);
    }

    public static Index getInstance(){
        if (instance == null) instance = new Index();
        return instance;
    }
}