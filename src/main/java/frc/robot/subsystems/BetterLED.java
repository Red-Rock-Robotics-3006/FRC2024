package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.util.Color;

public class BetterLED extends SubsystemBase{

    private static BetterLED instance = null;

    private AddressableLED control = new AddressableLED(Constants.LED.LED_CHANNEL_ID);
    private LEDBuffer buffer = new LEDBuffer(Constants.LED.NUM_LEDS);

    // private Intake intake = Intake.getInstance();
    // private Index index = Index.getInstance();
    // private Shooter shooter = Shooter.getInstance();
    // private SwerveIO swerve = TunerConstants.DriveTrain;

    private boolean initialAlliance = false;//TODO implement alliance detecting
    private enum State {RESTING, NOTE_DETECTED, HOMING_NOTE, HAS_NOTE, HOMING_APRILTAG}
    private State RobotState = State.RESTING;
    private int allianceColorCutoff = buffer.getLength() / 4;
    private int driveStateColorCutoff = buffer.getLength() * 3 / 4;
    private int driveStateControl = 0;

    private boolean policeModeEnabled = true;
    private int policeMode = 0;
    private int policeModeControl1 = 0;
    private int policeModeControl2 = 0;
    private int policeModeControl3 = 0;
    private int policeModeColorControl3 = 0;

    private Color RED = new Color(255, 0, 0);
    private Color BLUE = new Color(0, 0, 255);
    private Color WHITE = new Color(255, 255, 255);
    private Color OFF = new Color(0, 0, 0);
    

    /**
     * Constructor for LED which registers the subsystem and sets a specified portion of the LED lights to the alliance color
     */
    private BetterLED() {
        this.setName("LED");
        this.register();
        this.control.setLength(this.buffer.getLength());
        this.control.start();

        if (this.initialAlliance) {//true is red
            this.buffer.setLED(0, this.allianceColorCutoff, RED);
        }
        else {//false is blue
            this.buffer.setLED(0, this.allianceColorCutoff, BLUE);
        }
        this.control.setData(buffer);
    }

    public void setState(State s) {
        this.RobotState = s;
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
        System.out.println("police mode set to " + this.policeMode);
    }

    public void reset() {
        if (this.policeModeEnabled) this.togglePoliceMode();
        if (this.initialAlliance) {//true is red
            this.buffer.setLED(0, this.allianceColorCutoff, RED);
        }
        else {//false is blue
            this.buffer.setLED(0, this.allianceColorCutoff, RED);
        }
        this.buffer.setLED(this.allianceColorCutoff, driveStateColorCutoff, RED);
        this.control.setData(this.buffer);
    }

    public void setSection(int start, int end, int r, int g, int b) {
        this.buffer.setRGB(start, end, r, g, b);
        control.setData(buffer);
    }

    public void periodic() {
        // if (!this.policeModeEnabled) {
        //     switch(this.RobotState) { //TODO make the led patterns more intuitive (ie flashing instead of colors)
        //         case RESTING:
        //             this.setSection(this.allianceColorCutoff, this.driveStateColorCutoff, 0, 0, 0);//no color
        //             break;
        //         case NOTE_DETECTED:
        //             this.setSection(this.allianceColorCutoff, this.driveStateColorCutoff, 115, 81, 226);//cube purple
        //             break;
        //         case HOMING_NOTE:
        //             this.setSection(this.allianceColorCutoff, this.driveStateColorCutoff, 255, 183, 3);//cone yellow
        //             break;
        //         case HAS_NOTE:
        //             this.setSection(this.allianceColorCutoff, this.driveStateColorCutoff, 252, 85, 36);//note orange
        //             break;
        //         case HOMING_APRILTAG:
        //             this.setSection(this.allianceColorCutoff, this.driveStateColorCutoff, 30, 225, 30);//limelight green
        //             break;
        //     }
        // }

            // switch(this.swerve.getDriveState()) {
            //     case FIELD_CENTRIC:
            //         for (int i = this.driveStateColorCutoff; i < this.buffer.getLength(); i++) {
            //             this.buffer.setRGB(i, 255, 255, 255);
            //         }
            //         this.control.setData(this.buffer);
            //         break;
            //     case ROBOT_CENTRIC:
            //         this.driveStateControl++;
            //         if (this.driveStateControl % 50 == 0) {
            //             for (int i = this.driveStateColorCutoff; i < this.buffer.getLength(); i++) {
            //                 this.buffer.setRGB(i, 255, 255, 255);
            //             }
            //         }
            //         else if (this.driveStateControl % 25 == 0) {
            //             for (int i = this.driveStateColorCutoff; i < this.buffer.getLength(); i++) {
            //                 this.buffer.setRGB(i, 0, 0, 0);
            //             }
            //         }
            //         this.control.setData(this.buffer);
            //         break;
            // }
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
        // else if (!this.intake.noteDetected() && !this.index.hasNote()) {
        //     this.setState(State.RESTING);
        // }
        // else if (this.index.hasNote()) {
        //     this.setState(State.HAS_NOTE);
        // }
        //123, 231, 244 giorgio blue :)

        if (this.policeModeEnabled) {
            if (policeMode == 0) {//solid color
                this.buffer.setLED(0, buffer.getLength()/2, RED);
                this.buffer.setLED(buffer.getLength()/2, buffer.getLength(), BLUE);
                control.setData(buffer);
            }
            if (policeMode == 1) {//solid alternating color
                policeModeControl1++;
                if (policeModeControl1 % 50 == 0) {
                    this.buffer.setLED(RED);
                    control.setData(buffer);
                }
                else if (policeModeControl1 % 25 == 0) {
                    this.buffer.setLED(BLUE);
                    control.setData(buffer);
                }
            }
            else if (policeMode == 2) {//one flash each color on two halves
                policeModeControl2++;
                if (policeModeControl2 % 40 == 0) {
                    this.buffer.setLED(0, this.buffer.getLength()/2, RED);
                    this.buffer.setLED(this.buffer.getLength()/2, this.buffer.getLength(), OFF);
                    control.setData(buffer);
                }
                else if (policeModeControl2 % 20 == 0) {
                    this.buffer.setLED(0, this.buffer.getLength()/2, OFF);
                    this.buffer.setLED(this.buffer.getLength()/2, this.buffer.getLength(), BLUE);
                    control.setData(buffer);
                }
            }
            else if (policeMode == 3) {//three flashes each color on two halves
                policeModeControl3++;
                if (policeModeColorControl3 % 2 == 0) {
                    if (policeModeControl3 % 8 == 0) {
                        this.buffer.setLED(0, this.buffer.getLength()/2, OFF);
                        control.setData(buffer);
                    }

                    else if (policeModeControl3 % 4 == 0) {
                        this.buffer.setLED(0, this.buffer.getLength()/2, BLUE);
                        control.setData(buffer);


                        if (policeModeControl3 == 20) {
                            policeModeControl3 = 0;
                            policeModeColorControl3++;
                        }
                    }

                    this.buffer.setLED(this.buffer.getLength()/2, this.buffer.getLength(), OFF);
                    control.setData(buffer);
                }
                else {
                    if (policeModeControl3 % 8 == 0) {
                        this.buffer.setLED(this.buffer.getLength()/2, this.buffer.getLength(), OFF);
                        control.setData(buffer);
                    }

                    else if (policeModeControl3 % 4 == 0) {
                        this.buffer.setLED(this.buffer.getLength()/2, this.buffer.getLength(), RED);
                        control.setData(buffer);


                        if (policeModeControl3 == 20) {
                            policeModeControl3 = 0;
                            policeModeColorControl3++;
                        }
                    }

                    this.buffer.setLED(0, this.buffer.getLength()/2, OFF);
                    control.setData(buffer);
                }
            }
            else if (policeMode == 4) {//segmented flashes

            }
            else if(policeMode == 5) { // Fill
                policeModeControl2 = (policeModeControl2 + 1) % 40;
                int index = (int)(((policeModeControl2 % 20 + 1) / 20.0) * this.buffer.getLength()/2);
                if(policeModeControl2 % 20 == 0)
                    this.buffer.setLED(OFF);
                if (policeModeControl2 / 20 == 0) {
                    this.buffer.setLED(this.buffer.getLength()/2 - index, this.buffer.getLength()/2, RED);
                    control.setData(buffer);
                }
                else {
                    this.buffer.setLED(this.buffer.getLength()/2, this.buffer.getLength()/2 + index, BLUE);
                    control.setData(buffer);
                }
            }
            else if(policeMode == 6) { // Fill Gradient
                policeModeControl2 = (policeModeControl2 + 1) % 40;
                int index = (int)(((policeModeControl2 % 20 + 1) / 20.0) * this.buffer.getLength()/2);
                if(policeModeControl2 % 20 == 0)
                    this.buffer.setLED(OFF);
                if (policeModeControl2 / 20 == 0) {
                    this.buffer.setRGB(this.buffer.getLength()/2 - index, this.buffer.getLength()/2, n -> 255, n -> 0, n-> (this.buffer.getLength()/2-n)*6  );
                    control.setData(buffer);
                }
                else {
                    this.buffer.setRGB(this.buffer.getLength()/2, this.buffer.getLength()/2 + index, n -> (n-this.buffer.getLength()/2)*2, n -> 0, n-> 255);
                    control.setData(buffer);
                }
            }
        }
    }

    /**
     * Singleton architecture which returns the singular instance of LED
     * @return the instance (which is instantiated when first called)
     */
    public static BetterLED getInstance(){
        if (instance == null) instance = new BetterLED();
        return instance;
    }
}