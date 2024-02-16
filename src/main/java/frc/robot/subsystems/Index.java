package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
// import com.revrobotics.Rev2mDistanceSensor;
// import com.revrobotics.Rev2mDistanceSensor.Port;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;


public class Index extends SubsystemBase{
    
    private static Index instance = null;

    private final CANSparkFlex m_indexMotor = new CANSparkFlex(Constants.Index.INDEX_MOTOR_ID, CANSparkMax.MotorType.kBrushless);
    private final CANSparkFlex m_shooterLeftMotor = new CANSparkFlex(42, CANSparkMax.MotorType.kBrushless);
    private final CANSparkFlex m_shooterRightMotor = new CANSparkFlex(59, CANSparkMax.MotorType.kBrushless);
    //private final DigitalInput beamBrake = new DigitalInput(Constants.Index.SWITCH_CHANNEL_ID);
    // private final Rev2mDistanceSensor distanceSensor = new Rev2mDistanceSensor(Port.kOnboard);
    
    private boolean isTransferring = false;
    private double hasNoteThreshold = 10;//TODO tune this

    private Index() {
        this.setName("Index");
        this.register();

        this.m_indexMotor.restoreFactoryDefaults();
        this.m_indexMotor.setInverted(true);
        this.m_indexMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);

        this.m_shooterLeftMotor.restoreFactoryDefaults();
        this.m_shooterLeftMotor.setInverted(true);
        this.m_shooterLeftMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);

        this.m_shooterRightMotor.restoreFactoryDefaults();
        this.m_shooterRightMotor.setInverted(false);
        this.m_shooterRightMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);

        // this.distanceSensor.setAutomaticMode(true);
    }

    public void setTransferring(boolean b) {
        this.isTransferring = b;
    }

    public boolean getTransferring(boolean b) {
        return this.isTransferring;
    }

    public void setSpeed(double speed) {
        this.m_indexMotor.set(speed);
    }

    public void startTransfer() {
        this.isTransferring = true;
        this.setSpeed(0.3);//test this when possible
    }

    public void startOuttaking() {
        this.m_shooterRightMotor.set(0.2);
        this.m_shooterLeftMotor.set(0.2);
    }

    public void stopOuttaking() {
        this.m_shooterRightMotor.set(0);
        this.m_shooterLeftMotor.set(0);
    }

    public void stopTransfer() {
        this.isTransferring = false;
        this.setSpeed(0);
    }

    // public boolean hasNote() {
    //     if (this.beamBrake.get()) return true;
    //     return false;
    // }

    // public boolean hasNote2() {
    //     if (this.distanceSensor.getRange() <= this.hasNoteThreshold && this.distanceSensor.isRangeValid()) return true;
    //     return false;
    // }

    // public double getRange() {
    //     return this.distanceSensor.getRange();
    // }

    // boolean isDisplaying = false;
    // public void toggleDisplayRanges() {
    //     if (isDisplaying) isDisplaying = false;
    //     else isDisplaying = true;
    // }

    // public void periodic() {
    //     if (isDisplaying) {
    //         System.out.println(getRange());
    //     }
    // }

    /**
     * Singleton architecture which returns the singular instance of Index
     * @return the instance (which is instantiated when first called)
     */
    public static Index getInstance(){
        if (instance == null) instance = new Index();
        return instance;
    }
}