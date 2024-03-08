package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LED extends SubsystemBase{

    private static LED instance = null;

    private AddressableLED control = new AddressableLED(Constants.LED.LED_RIGHT_CHANNEL_ID);
    private AddressableLEDBuffer buffer = new AddressableLEDBuffer(Constants.LED.NUM_LEDS);

    private Index index = Index.getInstance();

    private enum State {RESTING, NOTE_DETECTED, HOMING_NOTE, HAS_NOTE, HOMING_APRILTAG}
    private State RobotState = State.RESTING;

    private int policeMode = 0;
    private int policeModeControl1 = 0;
    private int policeModeControl2 = 0;
    private int policeModeConfigControl2 = 0;

    /**
     * Constructor for LED which registers the subsystem and sets a specified portion of the LED lights to the alliance color
     */
    private LED() {
        this.setName("LED");
        this.register();
        this.control.setLength(this.buffer.getLength());

        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, 255, 165, 0);
        }
        this.control.setData(buffer);
        
        this.control.start();
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
        for (int i = 0; i < buffer.getLength(); i++) {
            this.buffer.setRGB(i, 255, 255, 255);
        }
        this.control.setData(this.buffer);
    }

    public void setLights(int r, int g, int b) {
        if (r > 255 || g > 255 || b > 255) {
            for (int i = 0; i < buffer.getLength(); i++) {
                this.buffer.setRGB(i, 255, 255, 255);
            }
            control.setData(buffer);
        }
        else {
            for (int i = 0; i < buffer.getLength(); i++) {
                this.buffer.setRGB(i, r, g, b);
            }
            control.setData(buffer);
        }
    }

    public void setLights(int start, int end, int r, int g, int b) {
        if (r > 255 || g > 255 || b > 255) {
            for (int i = start; i <= end; i++) {
                this.buffer.setRGB(i, 255, 255, 255);
            }
            control.setData(buffer);
        }
        else {
            for (int i = start; i <= end; i++) {
                this.buffer.setRGB(i, r, g, b);
            }
            control.setData(buffer);
        }
    }

    int blinkControl = 0;
    public void periodic() {
        if (index.hasNote()) this.setState(State.HAS_NOTE);
        else this.setState(State.RESTING);

        if (!Constants.Settings.POLICE_MODE_ENABLED) {
            switch(this.RobotState) {
                case RESTING:
                    this.setLights(255, 255, 255);
                    break;
                case HAS_NOTE:
                    blinkControl++;
                    if (blinkControl % 50 < 25) this.setLights(255, 0, 255);
                    else this.setLights(0, 0, 0);
                    break;

                //other states are currently unused
                case NOTE_DETECTED:
                    for (int i = 0; i < buffer.getLength(); i++) {
                        this.buffer.setRGB(i, 115, 81, 226);//cube purple
                    }
                    this.control.setData(this.buffer);
                    break;
                case HOMING_NOTE:
                    for (int i = 0; i < buffer.getLength(); i++) {
                        this.buffer.setRGB(i, 255, 183, 3);//cone yellow
                    }
                    this.control.setData(this.buffer);
                    break;
                
                case HOMING_APRILTAG:
                    for (int i = 0; i < buffer.getLength(); i++) {
                        this.buffer.setRGB(i, 30, 225, 30);//limelight green
                    }
                    this.control.setData(this.buffer);
                    break;
            }
        }
        else if (Constants.Settings.POLICE_MODE_ENABLED) {
            if (policeMode == 0) {//solid color
                this.setLights(0, buffer.getLength() / 2, 255, 0, 0);
                this.setLights(buffer.getLength() / 2, buffer.getLength(), 0, 0, 255);
            }
            if (policeMode == 1) {//solid alternating color
                policeModeControl1++;
                if (policeModeControl1 % 50 == 0) this.setLights(255, 0, 0);
                else if (policeModeControl1 % 25 == 0) this.setLights(0, 0, 255);
            }
            else if (policeMode == 2) {//really cool mode
                policeModeControl2++;
                if (policeModeConfigControl2 % 2 == 0) {
                    if (policeModeControl2 % 8 == 0) {
                        this.setLights(0, 2, 0, 0, 255);
                        this.setLights(3, 5, 255, 0, 0);
                        this.setLights(6, 8, 255, 255, 255);
                        this.setLights(9, 11, 0, 0, 255);
                        this.setLights(12, 14, 255, 0, 0);
                    }
                    else if (policeModeControl2 % 4 == 0) this.setLights(0, 0, 0);

                }
                else {
                    if (policeModeControl2 % 8 == 0) {
                        this.setLights(0, 2, 255, 0, 0);
                        this.setLights(3, 5, 0, 0, 255);
                        this.setLights(6, 8, 255, 255, 255);
                        this.setLights(9, 11, 255, 0, 0);
                        this.setLights(12, 14, 0, 0, 255);
                    }
                    else if (policeModeControl2 % 4 == 0) this.setLights(0, 0, 0);
                }
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