package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;


public class Index extends SubsystemBase{
    
    private static Index instance = null;

    private final CANSparkMax m_indexMotor = new CANSparkMax(Constants.Index.INDEX_MOTOR_ID, CANSparkMax.MotorType.kBrushless);
    private final DigitalInput beamBrake = new DigitalInput(Constants.Index.SWITCH_CHANNEL_ID);
    private final Rev2mDistanceSensor distanceSensor = new Rev2mDistanceSensor(Port.kOnboard);
    
    private boolean isTransferring = false;
    private double hasNoteThreshold = 10;//TODO tune this

    private Index() {
        this.setName("Index");
        this.register();

        this.m_indexMotor.restoreFactoryDefaults();
        this.m_indexMotor.setInverted(false);
        this.m_indexMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        this.distanceSensor.setAutomaticMode(true);
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
        this.setSpeed(0.1);//test this when possible
    }

    public void stopTransfer() {
        this.isTransferring = false;
        this.setSpeed(0);
    }

    // public boolean hasNote() {
    //     if (this.beamBrake.get()) return true;
    //     return false;
    // }

    public boolean hasNote2() {
        if (this.distanceSensor.getRange() <= this.hasNoteThreshold && this.distanceSensor.isRangeValid()) return true;
        return false;
    }

    public double getRange() {
        return this.distanceSensor.getRange();
    }

    boolean isDisplaying = false;
    public void toggleDisplayRanges() {
        if (isDisplaying) isDisplaying = false;
        else isDisplaying = true;
    }

    public void periodic() {
        if (isDisplaying) {
            System.out.println(getRange());
        }
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