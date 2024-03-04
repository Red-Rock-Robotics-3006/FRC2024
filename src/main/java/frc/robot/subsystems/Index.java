package frc.robot.subsystems;

import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;


public class Index extends SubsystemBase{
    
    private static Index instance = null;

    private final CANSparkMax m_indexMotor = new CANSparkMax(Constants.Index.INDEX_MOTOR_ID, CANSparkMax.MotorType.kBrushless);
    // private final DigitalInput intakeBeamBrake = new DigitalInput(Constants.Index.INTAKE_BB_SENSOR_ID);
    private final DigitalInput indexBeamBrake = new DigitalInput(Constants.Index.INDEX_BB_SENSOR_ID);
    // private final TimeOfFlight intakeTOFSensor = new TimeOfFlight(Constants.Index.INTAKE_TOF_SENSOR_ID);
    // private final TimeOfFlight indexTOFSensor = new TimeOfFlight(Constants.Index.INDEX_TOF_SENSOR_ID);
    
    private boolean isTransferring = false;
    // private double clearThresholdIntake;
    // private double clearThresholdIndex;
    // private double hasNoteThresholdDeviation = 10;//in mm, tune this

    private Index() {
        this.setName("Index");
        this.register();

        this.m_indexMotor.restoreFactoryDefaults();
        this.m_indexMotor.setInverted(false);
        this.m_indexMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        // this.intakeTOFSensor.setRangeOfInterest(8, 8, 12, 12);
        // this.intakeTOFSensor.setRangingMode(TimeOfFlight.RangingMode.Medium, 24);

        // this.indexTOFSensor.setRangeOfInterest(8, 8, 12, 12);
        // this.indexTOFSensor.setRangingMode(TimeOfFlight.RangingMode.Medium, 24);

        // this.clearThresholdIntake = this.intakeTOFSensor.getRange();
        // this.clearThresholdIndex = this.indexTOFSensor.getRange();
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

    public void reverseTransfer() {
        this.setSpeed(-0.2);
    }

    // public boolean noteInIndex() {
    //     return this.indexTOFSensor.getRange() < this.clearThresholdIndex - this.hasNoteThresholdDeviation && this.indexTOFSensor.isRangeValid();
    // }

    // public boolean noteInIntake() {
    //     return this.intakeTOFSensor.getRange() < this.clearThresholdIntake - this.hasNoteThresholdDeviation && this.intakeTOFSensor.isRangeValid();
    // }

    public boolean hasNote() {
        return !this.indexBeamBrake.get();
    }

    // public boolean hasNote2() {
        
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