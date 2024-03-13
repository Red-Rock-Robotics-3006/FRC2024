package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.index.TOFSensor;

public class LED extends SubsystemBase{

    private static LED instance = null;

    private AddressableLED control = new AddressableLED(Constants.LED.LED_CHANNEL_ID);
    private AddressableLEDBuffer buffer = new AddressableLEDBuffer(Constants.LED.NUM_LEDS);

    private TOFSensor sensor = TOFSensor.getInstance();

    private enum State {RESTING, NOTE_DETECTED, HOMING_NOTE, HAS_NOTE, HOMING_APRILTAG}
    private State RobotState = State.RESTING;

    private boolean policeModeEnabled = false;
    private int policeMode = 0;
    private int policeModeControl1 = 0;
    private int policeModeControl2 = 0;
    private int policeModeConfigControl2 = 0;

    private final Color NOTE_ORANGE = new Color(255, 15, 0);
    private final Color WHITE = new Color(255, 255, 255);
    private final Color BLUE = new Color(0, 0, 255);
    private final Color RED = new Color(255, 0, 0);
    private final Color OFF = new Color(0, 0, 0);

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
    }

    public void setLights(int r, int g, int b) {
        if (r > 255 || g > 255 || b > 255) {
            for (int i = 0; i < buffer.getLength(); i++) {
                this.buffer.setRGB(i, 255, 255, 255);
            }
        }
        else {
            for (int i = 0; i < buffer.getLength(); i++) {
                this.buffer.setRGB(i, r, g, b);
            }
        }
    }

    public void setLights(int start, int end, int r, int g, int b) {
        if (r > 255 || g > 255 || b > 255) {
            for (int i = start; i <= end; i++) {
                this.buffer.setRGB(i, 255, 255, 255);
            }
        }
        else {
            for (int i = start; i <= end; i++) {
                this.buffer.setRGB(i, r, g, b);
            }
        }
    }

    public void setLights(Color c) {
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setLED(i, c);
        }
    }

    public void setLights(int start, int end, Color c) {
        for (int i = start; i < end; i++) {
            buffer.setLED(i, c);
        }
    }

    public void togglePoliceModeEnabled() {
        if (policeModeEnabled) policeModeEnabled = false;
        else policeModeEnabled = true;
    }

    int blinkControl = 0;
    @SuppressWarnings("unused")
    public void periodic() {
        if (sensor.hasNote()) this.setState(State.HAS_NOTE);
        else this.setState(State.RESTING);

        if (!policeModeEnabled) {
            switch(this.RobotState) {
                case RESTING:
                    this.setLights(WHITE);
                    break;
                case HAS_NOTE:
                    blinkControl++;
                    if (blinkControl % 30 < 15) this.setLights(NOTE_ORANGE);
                    else this.setLights(OFF);
                    break;

                //other states are currently unused
                case NOTE_DETECTED:
                    for (int i = 0; i < buffer.getLength(); i++) {
                        this.buffer.setRGB(i, 115, 81, 226);//cube purple
                    }
                    break;
                case HOMING_NOTE:
                    for (int i = 0; i < buffer.getLength(); i++) {
                        this.buffer.setRGB(i, 255, 183, 3);//cone yellow
                    }
                    break;
                
                case HOMING_APRILTAG:
                    for (int i = 0; i < buffer.getLength(); i++) {
                        this.buffer.setRGB(i, 30, 225, 30);//limelight green
                    }
                    break;
            }
        }
        else if (Constants.Settings.POLICE_MODE_ENABLED && policeModeEnabled) {
            if (policeMode == 0) {//solid color
                this.setLights(0, buffer.getLength() / 2, RED);
                this.setLights(buffer.getLength() / 2, buffer.getLength(), BLUE);
            }
            if (policeMode == 1) {//solid alternating color
                policeModeControl1++;
                if (policeModeControl1 % 50 == 0) this.setLights(RED);
                else if (policeModeControl1 % 25 == 0) this.setLights(BLUE);
            }
            else if (policeMode == 2) {//really cool mode
                policeModeControl2++;
                if (policeModeConfigControl2 % 2 == 0) {
                    if (policeModeControl2 % 8 == 0) {
                        this.setLights(0, 3, BLUE);
                        this.setLights(3, 6, RED);
                        this.setLights(6, 9, WHITE);
                        this.setLights(9, 12, BLUE);
                        this.setLights(12, 15, RED);
                    }
                    else if (policeModeControl2 % 4 == 0) this.setLights(OFF);
                    if (policeModeControl2 == 16) {
                        policeModeConfigControl2++;
                        policeModeControl2 = 0;
                    }
                }
                else {
                    if (policeModeControl2 % 8 == 0) {
                        this.setLights(0, 3, RED);
                        this.setLights(3, 6, BLUE);
                        this.setLights(6, 9, WHITE);
                        this.setLights(9, 12, RED);
                        this.setLights(12, 15, BLUE);
                    }
                    else if (policeModeControl2 % 4 == 0) this.setLights(OFF);
                    if (policeModeControl2 == 16) {
                        policeModeConfigControl2++;
                        policeModeControl2 = 0;
                    }
                }
            }
        }
         
        this.control.setData(this.buffer);
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