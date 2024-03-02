package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveIO;
import frc.robot.subsystems.swerve.generated.TunerConstants;

public class LED extends SubsystemBase{

    private static LED instance = null;

    private AddressableLED controlRight = new AddressableLED(Constants.LED.LED_RIGHT_CHANNEL_ID);
    private AddressableLEDBuffer bufferRight = new AddressableLEDBuffer(Constants.LED.NUM_LEDS);

    private Intake intake = Intake.getInstance();
    private Index index = Index.getInstance();
    // private Shooter shooter = Shooter.getInstance();
    private SwerveIO swerve = TunerConstants.DriveTrain;

    private boolean initialAlliance = false;//TODO implement alliance detecting
    private enum State {RESTING, NOTE_DETECTED, HOMING_NOTE, HAS_NOTE, HOMING_APRILTAG}
    private State RobotState = State.RESTING;
    private int driveStateControl = 0;

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
        this.controlRight.setLength(this.bufferRight.getLength());

        for (int i = 0; i < bufferRight.getLength(); i++) {
            bufferRight.setRGB(i, 255, 165, 0);
        }
        this.controlRight.setData(bufferRight);
        
        this.controlRight.start();

        // if (this.initialAlliance) {//true is red
        //     for (int i = 0; i < 0; i++) {
        //         this.buffer.setRGB(i, 255, 0, 0);
        //     }
        // }
        // else {//false is blue
        //     for (int i = 0; i < 0; i++) {
        //         this.buffer.setRGB(i, 0, 0, 255);
        //     }
        // }
        // this.control.setData(buffer);
    }

    public void setState(State s) {
        this.RobotState = s;
    }

    public State getState(State s) {
        return this.RobotState;
    }

    public void setPoliceMode(int mode) {
        this.policeMode = mode;
    }

    boolean toggleHasNoteControl = false;
    public void toggleHasNote() {
        if (RobotState == State.HAS_NOTE) this.setState(State.RESTING);
        else this.setState(State.HAS_NOTE);
    }

    public void reset() {
        this.setState(State.RESTING);
        for (int i = 0; i < bufferRight.getLength(); i++) {
            this.bufferRight.setRGB(i, 255, 255, 255);
        }
        this.controlRight.setData(this.bufferRight);
    }

    int blinkControl = 0;
    public void periodic() {
        if (!Constants.Settings.POLICE_MODE_ENABLED) {
            switch(this.RobotState) { //TODO make the led patterns more intuitive (ie flashing instead of colors)
                case RESTING:
                    for (int i = 0; i < bufferRight.getLength(); i++) {
                        this.bufferRight.setRGB(i, 255, 255, 255);//white
                    }
                    this.controlRight.setData(this.bufferRight);
                    break;
                case NOTE_DETECTED:
                    for (int i = 0; i < bufferRight.getLength(); i++) {
                        this.bufferRight.setRGB(i, 115, 81, 226);//cube purple
                    }
                    this.controlRight.setData(this.bufferRight);
                    break;
                case HOMING_NOTE:
                    for (int i = 0; i < bufferRight.getLength(); i++) {
                        this.bufferRight.setRGB(i, 255, 183, 3);//cone yellow
                    }
                    this.controlRight.setData(this.bufferRight);
                    break;
                case HAS_NOTE:
                    blinkControl++;

                    if (blinkControl % 60 < 30) {
                        for (int i = 0; i < bufferRight.getLength(); i++) {
                            this.bufferRight.setRGB(i, 255, 0, 255);//red
                        }
                        this.controlRight.setData(this.bufferRight);
                    }
                    else {
                        for (int i = 0; i < bufferRight.getLength(); i++) {
                            this.bufferRight.setRGB(i, 0, 0, 0);//nothing
                        }
                        this.controlRight.setData(this.bufferRight);
                    }
                    break;
                case HOMING_APRILTAG:
                    for (int i = 0; i < bufferRight.getLength(); i++) {
                        this.bufferRight.setRGB(i, 30, 225, 30);//limelight green
                    }
                    this.controlRight.setData(this.bufferRight);
                    break;
            }
        }
        else if (Constants.Settings.POLICE_MODE_ENABLED) {
            if (policeMode == 0) {//solid color
                for (int i = 0; i < bufferRight.getLength() / 2; i++) {
                    bufferRight.setRGB(i, 255, 0, 0);
                }
                for (int i = bufferRight.getLength() / 2; i < bufferRight.getLength(); i++) {
                    bufferRight.setRGB(i, 0, 0, 255);
                }
                controlRight.setData(bufferRight);
            }
            if (policeMode == 1) {//solid alternating color
                policeModeControl1++;
                if (policeModeControl1 % 50 == 0) {
                    for (int i = 0; i < bufferRight.getLength(); i++) {
                        bufferRight.setRGB(i, 255, 0, 0);
                    }
                    controlRight.setData(bufferRight);
                }
                else if (policeModeControl1 % 25 == 0) {
                    for (int i = 0; i < bufferRight.getLength(); i++) {
                        bufferRight.setRGB(i, 0, 0, 255);
                    }
                    controlRight.setData(bufferRight);
                }
            }
            else if (policeMode == 2) {//one flash each color on two halves
                policeModeControl2++;
                if (policeModeControl2 % 40 == 0) {
                    for (int i = 0; i < bufferRight.getLength() / 2; i++) {
                        bufferRight.setRGB(i, 255, 0, 0);
                    }
                    for (int i = bufferRight.getLength() / 2; i < bufferRight.getLength(); i++) {
                        bufferRight.setRGB(i, 0, 0, 0);
                    }
                    controlRight.setData(bufferRight);
                }
                else if (policeModeControl2 % 20 == 0) {
                    for (int i = 0; i < bufferRight.getLength() / 2; i++) {
                        bufferRight.setRGB(i, 0, 0, 0);
                    }
                    for (int i = bufferRight.getLength() / 2; i < bufferRight.getLength(); i++) {
                        bufferRight.setRGB(i, 0, 0, 255);
                    }
                    controlRight.setData(bufferRight);
                }
            }
            else if (policeMode == 3) {//three flashes each color on two halves
                policeModeControl3++;
                if (policeModeColorControl3 % 2 == 0) {
                    if (policeModeControl3 % 8 == 0) {
                        for (int i = 0; i < bufferRight.getLength() / 2; i++) {
                            bufferRight.setRGB(i, 0, 0, 0);
                        }
                        controlRight.setData(bufferRight);
                    }

                    else if (policeModeControl3 % 4 == 0) {
                        for (int i = 0; i < bufferRight.getLength() / 2; i++) {
                            bufferRight.setRGB(i, 0, 0, 255);
                        }
                        controlRight.setData(bufferRight);


                        if (policeModeControl3 == 20) {
                            policeModeControl3 = 0;
                            policeModeColorControl3++;
                        }
                    }

                    for (int i = bufferRight.getLength() / 2; i < bufferRight.getLength(); i++) {
                        bufferRight.setRGB(i, 0, 0, 0);
                    }
                    controlRight.setData(bufferRight);
                }
                else {
                    if (policeModeControl3 % 8 == 0) {
                        for (int i = bufferRight.getLength() / 2; i < bufferRight.getLength(); i++) {
                            bufferRight.setRGB(i, 0, 0, 0);
                        }
                        controlRight.setData(bufferRight);
                    }

                    else if (policeModeControl3 % 4 == 0) {
                        for (int i = bufferRight.getLength() / 2; i < bufferRight.getLength(); i++) {
                            bufferRight.setRGB(i, 255, 0, 0);
                        }
                        controlRight.setData(bufferRight);


                        if (policeModeControl3 == 20) {
                            policeModeControl3 = 0;
                            policeModeColorControl3++;
                        }
                    }

                    for (int i = 0; i < bufferRight.getLength() / 2; i++) {
                        bufferRight.setRGB(i, 0, 0, 0);
                    }
                    controlRight.setData(bufferRight);
                }
            }
            else if (policeMode == 4) {//segmented flashes

            }
        }

        if (index.hasNote()) {
            this.setState(State.HAS_NOTE);
        }
        else this.setState(State.RESTING);

        //     switch(this.swerve.getDriveState()) {
        //         case FIELD_CENTRIC:
        //             for (int i = bufferRight.getLength(); i < this.buffer.getLength(); i++) {
        //                 this.buffer.setRGB(i, 255, 255, 255);
        //             }
        //             this.control.setData(this.buffer);
        //             break;
        //         case ROBOT_CENTRIC:
        //             this.driveStateControl++;
        //             if (this.driveStateControl % 50 == 0) {
        //                 for (int i = bufferRight.getLength(); i < this.buffer.getLength(); i++) {
        //                     this.buffer.setRGB(i, 255, 255, 255);
        //                 }
        //             }
        //             else if (this.driveStateControl % 25 == 0) {
        //                 for (int i = bufferRight.getLength(); i < this.buffer.getLength(); i++) {
        //                     this.buffer.setRGB(i, 0, 0, 0);
        //                 }
        //             }
        //             this.control.setData(this.buffer);
        //             break;
        //     }
        // }
        
        // if (this.shooter.getHoming()) {
        //     this.setState(State.HOMING_APRILTAG);
        // }
        // else if (this.intake.noteDetected() && this.intake.getHoming()) {
        //     this.setState(State.HOMING_NOTE);
        // }
        // else if (this.intake.noteDetected()) {
        //     this.setState(State.NOTE_DETECTED);
        // }
        // if (!this.index.hasNote()) {
        //     this.setState(State.RESTING);
        // }
        // else if (this.index.hasNote()) {
        //     this.setState(State.HAS_NOTE);
        // }
        //123, 231, 244 giorgio blue :)

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