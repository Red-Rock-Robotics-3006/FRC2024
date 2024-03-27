package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.index.TOFSensor;
import frc.robot.subsystems.shooter.Shooter;

public class LED extends SubsystemBase{

    private static LED instance = null;

    private AddressableLED control = new AddressableLED(Constants.LED.LED_CHANNEL_ID);
    private AddressableLEDBuffer buffer = new AddressableLEDBuffer(Constants.LED.NUM_LEDS);

    private TOFSensor sensor = TOFSensor.getInstance();
    private Shooter shooter = Shooter.getInstance();

    private State RobotState = State.RESTING;

    private boolean policeModeEnabled = false;
    private int policeMode = 0;
    private int policeModeControl1 = 0;
    private int policeModeControl2 = 0;
    private int policeModeColorControl2 = 0;

    private boolean isAmping = false;
    private boolean autoEnd = false;

    private final Color NOTE_ORANGE = new Color(255, 15, 0);
    private final Color INIT_YELLOW = new Color(255, 165, 0);
    private final Color GREEN = new Color(0, 255, 0);
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

        this.setLights(INIT_YELLOW);
        this.control.setData(buffer);
        
        this.control.start();
    }

    public void setIsAmping(boolean b) {
        isAmping = b;
    }

    public void setAutoEnd(boolean b) {//TODO THIS IS FOR TEST
        autoEnd = b;
    }

    public void setState(State s) {
        this.RobotState = s;
        if (s == State.SCORING_AMP) isAmping = true;
    }

    public State getState(State s) {
        return this.RobotState;
    }

    public void setPoliceMode(int mode) {
        if (mode > 2) this.policeMode = 0;
        else this.policeMode = mode;
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
            for (int i = start; i < end; i++) {
                this.buffer.setRGB(i, 255, 255, 255);
            }
        }
        else {
            for (int i = start; i < end; i++) {
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
        else if (Constants.Settings.POLICE_MODE_ENABLED) policeModeEnabled = true;
    }

    int blinkControl = 0;
    @SuppressWarnings("unused")
    public void periodic() {
        if (autoEnd) this.setState(State.AUTO_END);
        else if (isAmping) this.setState(State.SCORING_AMP);
        else if (shooter.getHoming() && shooter.inRange()) this.setState(State.AUTO_AIM);
        else if (sensor.hasNote()) this.setState(State.HAS_NOTE);
        else this.setState(State.RESTING);

        if (!policeModeEnabled) {
            switch(this.RobotState) {
                case RESTING:
                    this.setLights(WHITE);
                    break;
                case HAS_NOTE:
                    blinkControl++;
                    if (blinkControl % 14 < 7) this.setLights(NOTE_ORANGE);
                    else this.setLights(OFF);
                    break;
                case AUTO_AIM:
                    blinkControl++; 
                    if (blinkControl % 6 < 3) this.setLights(GREEN);
                    else this.setLights(OFF);
                    break;
                case SCORING_AMP:
                    blinkControl++;
                    if (blinkControl % 6 < 3) this.setLights(BLUE);
                    else this.setLights(OFF); 
                    break;
                case AUTO_END:
                    this.setLights(BLUE);
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
            else if (policeMode == 2) {//three flashes each color on two halves
                policeModeControl2++;
                if (policeModeColorControl2 % 2 == 0) {
                    if (policeModeControl2 % 8 == 0) {
                        this.setLights(0, 8, BLUE);
                        this.setLights(16, 24, BLUE);
                    }
                    else if (policeModeControl2 % 4 == 0) this.setLights(OFF);
                    if (policeModeControl2 == 16) {
                        policeModeColorControl2++;
                        policeModeControl2 = 0;
                    }
                }
                else {
                    if (policeModeControl2 % 8 == 0) {
                        this.setLights(8, 16, RED);
                        this.setLights(24, buffer.getLength(), RED);
                    }
                    else if (policeModeControl2 % 4 == 0) this.setLights(OFF);
                    if (policeModeControl2 == 16) {
                        policeModeColorControl2++;
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