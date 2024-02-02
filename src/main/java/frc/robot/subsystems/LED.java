package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LED extends SubsystemBase{

    private static LED instance = null;

    private AddressableLED control = new AddressableLED(Constants.LED.LED_CHANNEL_ID);
    private AddressableLEDBuffer buffer = new AddressableLEDBuffer(Constants.LED.NUM_LEDS);

    private boolean initialAlliance = false;//TODO implement this
    private boolean restingState = false;
    private boolean noteDetectedState = false;
    private boolean homingNoteState = false;
    private boolean hasNoteState = false;
    private boolean homingAprilTagState = false;
    private boolean policeMode = false;

    private int allianceColorCutoff = buffer.getLength() / 3;

    /**
     * Constructor for LED which registers the subsystem and sets a specified portion of the LED lights to the alliance color
     */
    private LED() {
        this.setName("LED");
        this.register();

        control.setLength(buffer.getLength());
        if (initialAlliance) {
            for (int i = 0; i < allianceColorCutoff; i++) {
                buffer.setRGB(i, 255, 0, 0);
            }
        }
        else {
            for (int i = 0; i < allianceColorCutoff; i++) {
                buffer.setRGB(i, 0, 0, 255);
            }
        }
        control.setData(buffer);
    }

    /**
     * Sets restingState to specified parameter whilst setting all other state booleans to false
     * @param b user specified boolean which restingState is set to
     */
    public void setRestingState(boolean b) {
        this.restingState = b;

        this.noteDetectedState = !b;
        this.homingNoteState = !b;
        this.hasNoteState = !b;
        this.homingAprilTagState = !b;
    }

    /**
     * Sets noteDetectedState to specified parameter whilst setting all other state booleans to false
     * @param b user specified boolean which noteDetectedState is set to
     */
    public void setNoteDetectedState(boolean b) {
        this.noteDetectedState = b;

        this.restingState = !b;
        this.homingNoteState = !b;
        this.hasNoteState = !b;
        this.homingAprilTagState = !b;
    }

    /**
     * Sets homingNoteState to specified parameter whilst setting all other state booleans to false
     * @param b user specified boolean which homingNoteState is set to
     */
    public void setHomingNoteState(boolean b) {
        this.homingNoteState = b;

        this.restingState = !b;
        this.noteDetectedState = !b;
        this.hasNoteState = !b;
        this.homingAprilTagState = !b;
    }

    /**
     * Sets hasNoteState to specified parameter whilst setting all other state booleans to false
     * @param b user specified boolean which hasNoteState is set to
     */
    public void setHasNoteState(boolean b) {
        this.hasNoteState = b;

        this.restingState = !b;
        this.noteDetectedState = !b;
        this.homingNoteState = !b;
        this.homingAprilTagState = !b;
    }

    /**
     * Sets homingAprilTagState to specified parameter whilst setting all other state booleans to false
     * @param b user specified boolean which homingAprilTagState is set to
     */
    public void setHomingAprilTagState(boolean b) {
        this.homingAprilTagState = b;

        this.restingState = !b;
        this.noteDetectedState = !b;
        this.homingNoteState = !b;
        this.hasNoteState = !b;
    }

    public void periodic() {
        if (restingState) {
            for (int i = allianceColorCutoff; i < buffer.getLength(); i++) {
                buffer.setRGB(i, 255, 255, 255);//white
            }
            control.setData(buffer);
        }
        else if (noteDetectedState) {
            for (int i = allianceColorCutoff; i < buffer.getLength(); i++) {
                buffer.setRGB(i, 115, 81, 226);//cube purple
            }
            control.setData(buffer);
        }
        else if (homingNoteState) {
            for (int i = allianceColorCutoff; i < buffer.getLength(); i++) {
                buffer.setRGB(i, 255, 183, 3);//cone yellow
            }
            control.setData(buffer);
        }
        else if (hasNoteState) {
            for (int i = allianceColorCutoff; i < buffer.getLength(); i++) {
                buffer.setRGB(i, 252, 85, 36);//note orange
            }
            control.setData(buffer);
        }
        else if (homingAprilTagState) {
            for (int i = allianceColorCutoff; i < buffer.getLength(); i++) {
                buffer.setRGB(i, 30, 225, 30);//limelight green
            }
            control.setData(buffer);
        }

        else if (policeMode) {
            int policeModeControl = 0;
            int policeModeColorControl = 0;
            policeModeControl++;
            if (policeModeColorControl % 2 == 0) {
                if (policeModeControl % 8 == 0) {
                    for (int i = 0; i < buffer.getLength() / 2; i++) {
                        buffer.setRGB(i, 0, 0, 0);
                    }
                    control.setData(buffer);
                }

                else if (policeModeControl % 4 == 0) {
                    for (int i = 0; i < buffer.getLength() / 2; i++) {
                        buffer.setRGB(i, 0, 0, 255);
                    }
                    control.setData(buffer);


                    if (policeModeControl == 20) {
                        policeModeControl = 0;
                        policeModeColorControl++;
                    }
                }

                for (int i = buffer.getLength() / 2; i < buffer.getLength(); i++) {
                    buffer.setRGB(i, 0, 0, 0);
                }
                control.setData(buffer);
            }
            else {
                if (policeModeControl % 8 == 0) {
                    for (int i = buffer.getLength() / 2; i < buffer.getLength(); i++) {
                        buffer.setRGB(i, 0, 0, 0);
                    }
                    control.setData(buffer);
                }

                else if (policeModeControl % 4 == 0) {
                    for (int i = buffer.getLength() / 2; i < buffer.getLength(); i++) {
                        buffer.setRGB(i, 255, 0, 0);
                    }
                    control.setData(buffer);


                    if (policeModeControl == 20) {
                        policeModeControl = 0;
                        policeModeColorControl++;
                    }
                }

                for (int i = 0; i < buffer.getLength() / 2; i++) {
                    buffer.setRGB(i, 0, 0, 0);
                }
                control.setData(buffer);
            }
        }

        if (Intake.getInstance().noteDetected() && Intake.getInstance().getHoming()) {
            this.setHomingNoteState(true);
        }
        else if (Intake.getInstance().noteDetected()) {
            this.setNoteDetectedState(true);
        }
        else if (!Intake.getInstance().noteDetected() && !Index.getInstance().hasNote()) {
            this.setRestingState(true);
        }
        else if (Index.getInstance().hasNote()) {
            this.setHasNoteState(true);
        }
        else if (Shooter.getInstance().getHoming()) {
            this.setHomingAprilTagState(true);
        }
    
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