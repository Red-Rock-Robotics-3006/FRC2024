package frc.robot.subsystems.index;

import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TOFSensor extends SubsystemBase {

    private static TOFSensor instance = null;
    
    private final TimeOfFlight indexTOFSensor = new TimeOfFlight(Constants.Index.INDEX_TOF_SENSOR_ID);

    private double clearThresholdIndex = 400; // 300
    private double hasNoteThresholdDeviation = 30; // 100

    private TOFSensor() {
        super("TOFSensor");
        
        this.indexTOFSensor.setRangeOfInterest(8, 8, 12, 12);
        this.indexTOFSensor.setRangingMode(TimeOfFlight.RangingMode.Short, 24);
    }

    public boolean noteInIndex() {
        return this.indexTOFSensor.getRange() < this.clearThresholdIndex - this.hasNoteThresholdDeviation;
    }

    public boolean hasNote() {
        return this.noteInIndex();
    }

    public void periodic() {
        SmartDashboard.putNumber("TOF Range ", this.indexTOFSensor.getRange());
        SmartDashboard.putBoolean("Has note ", this.hasNote());
    }

    public static TOFSensor getInstance(){
        if (instance == null) instance = new TOFSensor();
        return instance;
    }
}
