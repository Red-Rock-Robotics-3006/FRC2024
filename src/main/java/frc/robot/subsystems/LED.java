package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveIO;
import frc.robot.subsystems.swerve.generated.TunerConstants;

public class LED extends SubsystemBase{

    private static LED instance = null;

    private AddressableLED control = new AddressableLED(Constants.LED.LED_CHANNEL_ID);
    private AddressableLEDBuffer buffer = new AddressableLEDBuffer(Constants.LED.NUM_LEDS);

    private Intake intake = Intake.getInstance();
    private Index index = Index.getInstance();
    private Shooter shooter = Shooter.getInstance();
    private SwerveIO swerve = TunerConstants.DriveTrain;

    private boolean initialAlliance = false;//TODO implement alliance detecting
    private enum State {RESTING, NOTE_DETECTED, HOMING_NOTE, HAS_NOTE, HOMING_APRILTAG}
    private State RobotState = State.RESTING;
    private int allianceColorCutoff = buffer.getLength() / 4;
    private int driveStateColorCutoff = buffer.getLength() * 3 / 4;
    private int driveStateControl = 0;

    private boolean policeModeEnabled = false;
    private int policeMode = 0;
    private int policeModeControl1 = 0;
    private int policeModeControl2 = 0;
    private int policeModeControl3 = 0;
    private int policeModeColorControl3 = 0;

    /**
     * Constructor for LED which registers the subsystem and sets a specified portion of the LED lights to the alliance color
     */
    private LED() {
        this.setName("LED");
        this.register();
        this.control.setLength(this.buffer.getLength());
        this.control.start();

        if (this.initialAlliance) {//true is red
            for (int i = 0; i < this.allianceColorCutoff; i++) {
                this.buffer.setRGB(i, 255, 0, 0);
            }
        }
        else {//false is blue
            for (int i = 0; i < this.allianceColorCutoff; i++) {
                this.buffer.setRGB(i, 0, 0, 255);
            }
        }
        this.control.setData(buffer);
    }

    public void setState(State s) {
        switch(s) {
            case RESTING: this.RobotState = State.RESTING;
            case NOTE_DETECTED: this.RobotState = State.NOTE_DETECTED;
            case HOMING_NOTE: this.RobotState = State.HOMING_NOTE;
            case HAS_NOTE: this.RobotState = State.HAS_NOTE;
            case HOMING_APRILTAG: this.RobotState = State.HOMING_APRILTAG;
        }
    }

    public State getState(State s) {
        return this.RobotState;
    }

    public void togglePoliceMode() {
        if (this.policeModeEnabled) this.policeModeEnabled = false;
        else this.policeModeEnabled = true;
    }

    public void setPoliceMode(int mode) {
        this.policeMode = mode;
    }

    public void reset() {
        if (this.policeModeEnabled) this.togglePoliceMode();
        if (this.initialAlliance) {//true is red
            for (int i = 0; i < this.allianceColorCutoff; i++) {
                this.buffer.setRGB(i, 255, 0, 0);
            }
        }
        else {//false is blue
            for (int i = 0; i < this.allianceColorCutoff; i++) {
                this.buffer.setRGB(i, 0, 0, 255);
            }
        }
        for (int i = this.allianceColorCutoff; i < driveStateColorCutoff; i++) {
            this.buffer.setRGB(i, 255, 255, 255);//white
        }
        this.control.setData(this.buffer);
    }

    public void periodic() {
        if (!this.policeModeEnabled) {
            switch(this.RobotState) { //TODO make the led patterns more intuitive (ie flashing instead of colors)
                case RESTING:
                    for (int i = this.allianceColorCutoff; i < this.driveStateColorCutoff; i++) {
                        this.buffer.setRGB(i, 255, 255, 255);//white
                    }
                    this.control.setData(this.buffer);
                    break;
                case NOTE_DETECTED:
                    for (int i = this.allianceColorCutoff; i < this.driveStateColorCutoff; i++) {
                        this.buffer.setRGB(i, 115, 81, 226);//cube purple
                    }
                    this.control.setData(this.buffer);
                    break;
                case HOMING_NOTE:
                    for (int i = this.allianceColorCutoff; i < this.driveStateColorCutoff; i++) {
                        this.buffer.setRGB(i, 255, 183, 3);//cone yellow
                    }
                    this.control.setData(this.buffer);
                    break;
                case HAS_NOTE:
                    for (int i = this.allianceColorCutoff; i < this.driveStateColorCutoff; i++) {
                        this.buffer.setRGB(i, 252, 85, 36);//note orange
                    }
                    this.control.setData(this.buffer);
                    break;
                case HOMING_APRILTAG:
                    for (int i = this.allianceColorCutoff; i < this.driveStateColorCutoff; i++) {
                        this.buffer.setRGB(i, 30, 225, 30);//limelight green
                    }
                    this.control.setData(this.buffer);
                    break;
            }

            switch(this.swerve.getDriveState()) {
                case FIELD_CENTRIC:
                    for (int i = this.driveStateColorCutoff; i < this.buffer.getLength(); i++) {
                        this.buffer.setRGB(i, 255, 255, 255);
                    }
                    this.control.setData(this.buffer);
                    break;
                case ROBOT_CENTRIC:
                    this.driveStateControl++;
                    if (this.driveStateControl % 50 == 0) {
                        for (int i = this.driveStateColorCutoff; i < this.buffer.getLength(); i++) {
                            this.buffer.setRGB(i, 255, 255, 255);
                        }
                    }
                    else if (this.driveStateControl % 25 == 0) {
                        for (int i = this.driveStateColorCutoff; i < this.buffer.getLength(); i++) {
                            this.buffer.setRGB(i, 0, 0, 0);
                        }
                    }
                    this.control.setData(this.buffer);
                    break;
            }
        }
        
        if (this.shooter.getHoming()) {
            this.setState(State.HOMING_APRILTAG);
        }
        else if (this.intake.noteDetected() && this.intake.getHoming()) {
            this.setState(State.HOMING_NOTE);
        }
        else if (this.intake.noteDetected()) {
            this.setState(State.NOTE_DETECTED);
        }
        else if (!this.intake.noteDetected() && !this.index.hasNote()) {
            this.setState(State.RESTING);
        }
        else if (this.index.hasNote()) {
            this.setState(State.HAS_NOTE);
        }
        //123, 231, 244 giorgio blue :)

        if (this.policeModeEnabled) {
            if (policeMode == 0) {//solid color
                for (int i = 0; i < buffer.getLength() / 2; i++) {
                    buffer.setRGB(i, 255, 0, 0);
                }
                for (int i = buffer.getLength() / 2; i < buffer.getLength(); i++) {
                    buffer.setRGB(i, 0, 0, 255);
                }
                control.setData(buffer);
            }
            if (policeMode == 1) {//solid alternating color
                policeModeControl1++;
                if (policeModeControl1 % 50 == 0) {
                    for (int i = 0; i < buffer.getLength(); i++) {
                        buffer.setRGB(i, 255, 0, 0);
                    }
                    control.setData(buffer);
                }
                else if (policeModeControl1 % 25 == 0) {
                    for (int i = 0; i < buffer.getLength(); i++) {
                        buffer.setRGB(i, 0, 0, 255);
                    }
                    control.setData(buffer);
                }
            }
            else if (policeMode == 2) {//one flash each color on two halves
                policeModeControl2++;
                if (policeModeControl2 % 40 == 0) {
                    for (int i = 0; i < buffer.getLength() / 2; i++) {
                        buffer.setRGB(i, 255, 0, 0);
                    }
                    for (int i = buffer.getLength() / 2; i < buffer.getLength(); i++) {
                        buffer.setRGB(i, 0, 0, 0);
                    }
                    control.setData(buffer);
                }
                else if (policeModeControl2 % 20 == 0) {
                    for (int i = 0; i < buffer.getLength() / 2; i++) {
                        buffer.setRGB(i, 0, 0, 0);
                    }
                    for (int i = buffer.getLength() / 2; i < buffer.getLength(); i++) {
                        buffer.setRGB(i, 0, 0, 255);
                    }
                    control.setData(buffer);
                }
            }
            else if (policeMode == 3) {//three flashes each color on two halves
                policeModeControl3++;
                if (policeModeColorControl3 % 2 == 0) {
                    if (policeModeControl3 % 8 == 0) {
                        for (int i = 0; i < buffer.getLength() / 2; i++) {
                            buffer.setRGB(i, 0, 0, 0);
                        }
                        control.setData(buffer);
                    }

                    else if (policeModeControl3 % 4 == 0) {
                        for (int i = 0; i < buffer.getLength() / 2; i++) {
                            buffer.setRGB(i, 0, 0, 255);
                        }
                        control.setData(buffer);


                        if (policeModeControl3 == 20) {
                            policeModeControl3 = 0;
                            policeModeColorControl3++;
                        }
                    }

                    for (int i = buffer.getLength() / 2; i < buffer.getLength(); i++) {
                        buffer.setRGB(i, 0, 0, 0);
                    }
                    control.setData(buffer);
                }
                else {
                    if (policeModeControl3 % 8 == 0) {
                        for (int i = buffer.getLength() / 2; i < buffer.getLength(); i++) {
                            buffer.setRGB(i, 0, 0, 0);
                        }
                        control.setData(buffer);
                    }

                    else if (policeModeControl3 % 4 == 0) {
                        for (int i = buffer.getLength() / 2; i < buffer.getLength(); i++) {
                            buffer.setRGB(i, 255, 0, 0);
                        }
                        control.setData(buffer);


                        if (policeModeControl3 == 20) {
                            policeModeControl3 = 0;
                            policeModeColorControl3++;
                        }
                    }

                    for (int i = 0; i < buffer.getLength() / 2; i++) {
                        buffer.setRGB(i, 0, 0, 0);
                    }
                    control.setData(buffer);
                }
            }
            else if (policeMode == 4) {//segmented flashes

            }
        }
    }

    /**
     * Singleton architecture which returns the singular instance of LED
     * @return the instance (which is instantiated when first called)
     */
    public static LED getInstance(){
        if (instance == null) instance = new LED();
        return instance;
    }
}