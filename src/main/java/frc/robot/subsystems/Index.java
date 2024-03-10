// package frc.robot.subsystems;

// import com.revrobotics.CANSparkFlex;
// import com.revrobotics.CANSparkMax;

// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import frc.robot.Constants;


// public class Index extends SubsystemBase{
    
//     private static Index instance = null;

//     private final CANSparkFlex m_indexMotor = new CANSparkFlex(Constants.Index.INDEX_MOTOR_ID, CANSparkMax.MotorType.kBrushless);
//     private final DigitalInput beamBrake = new DigitalInput(Constants.Index.SWITCH_CHANNEL_ID);
    
//     private boolean isTransferring = false;

//     private double transferSpeed = 0.85;

//     private Index() {
//         this.setName("Index");
//         this.register();

//         this.m_indexMotor.restoreFactoryDefaults();
//         this.m_indexMotor.setInverted(true);
//         this.m_indexMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
//     }

//     public void setTransferring(boolean b) {
//         this.isTransferring = b;
//     }

//     public boolean getTransferring(boolean b) {
//         return this.isTransferring;
//     }

//     public void setSpeed(double speed) {
//         this.m_indexMotor.set(speed);
//     }

//     public void startTransfer() {
//         this.setSpeed(this.transferSpeed);
//     }

//     public void reverseTransfer() {
//         this.setSpeed(-0.2);
//     }

//     public void stopTransfer() {
//         // this.isTransferring = false;
//         this.setSpeed(0);
//     }

//     // public boolean hasNote() {
//     //     return !this.beamBrake.get();
//     // }
    
//     public boolean

//     double current = 0;
//     public void periodic() {
//         // // if (this.isTransferring && this.hasNote())
//         // // {
//         // //     this.stopTransfer();
//         // //     SmartDashboard.putBoolean("Stopped",true);
//         // // }
//         //     SmartDashboard.putBoolean("Has Note", this.hasNote());
//         // System.out.println(this.hasNote());



//     }

//     /**
//      * Singleton architecture which returns the singular instance of Index
//      * @return the instance (which is instantiated when first called)
//      */
//     public static Index getInstance(){
//         if (instance == null) instance = new Index();
//         return instance;
//     }
// }

package frc.robot.subsystems;

import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
// import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;


public class Index extends SubsystemBase{

    public static final double kReverseTime = 0.1;
    
    private static Index instance = null;

    private final CANSparkFlex m_indexMotor = new CANSparkFlex(Constants.Index.INDEX_MOTOR_ID, CANSparkMax.MotorType.kBrushless);
    private final TimeOfFlight indexTOFSensor = new TimeOfFlight(Constants.Index.INDEX_TOF_SENSOR_ID);
    
    private boolean isTransferring = false;
    private double clearThresholdIndex;
    private double hasNoteThresholdDeviation = 100;//in mm, tune this

    private Index() {
        this.setName("Index");
        this.register();

        this.m_indexMotor.restoreFactoryDefaults();
        this.m_indexMotor.setInverted(true);
        this.m_indexMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        this.m_indexMotor.burnFlash();

        this.indexTOFSensor.setRangeOfInterest(8, 8, 12, 12);
        this.indexTOFSensor.setRangingMode(TimeOfFlight.RangingMode.Short, 24);

        this.clearThresholdIndex = 300;

        SmartDashboard.putNumber("index reverse time", kReverseTime);
    }

    public void setTransferring(boolean b) {
        this.isTransferring = b;
    }

    public boolean getTransferring() {
        return this.isTransferring;
    }

    public void setSpeed(double speed) {
        this.m_indexMotor.set(speed);
    }

    public void startTransfer() {
        this.setSpeed(0.85);
    }

    public void stopTransfer() {
        this.isTransferring = false;
        this.setSpeed(0);
    }

    public void reverseTransfer() {
        this.setSpeed(-0.2);
    }

    public void reverseFastTransfer() {
        this.setSpeed(-0.5);
    }

    public boolean noteInIndex() {
        return this.indexTOFSensor.getRange() < this.clearThresholdIndex - this.hasNoteThresholdDeviation;
    }

    public boolean hasNote() {
        return this.noteInIndex();
    }

    boolean stopped = false;

    public void periodic() {
        SmartDashboard.putNumber("TOF Range ", this.indexTOFSensor.getRange());
        SmartDashboard.putBoolean("Has note ", this.hasNote());
        // SmartDashboard.putBoolean("IS TRANSFERRING ", this.getTrPansferring());

        if (this.hasNote() && this.getTransferring()) {
            Intake.getInstance().setHoming(false);
            Intake.getInstance().stopIntake();
            this.stopTransfer();
            stopped = true;

            // CommandScheduler.getInstance().schedule(new StartEndCommand(
            //     () -> this.reverseFastTransfer(), 
            //     () -> this.stopTransfer(),
            //     this
            //     ).withTimeout(SmartDashboard.getNumber("index reverse time", kReverseTime))
            // );
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