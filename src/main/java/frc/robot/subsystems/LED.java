package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Arrays;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Intake;
import frc.robot.subsystems.swerve.SwerveIO;
import frc.robot.subsystems.swerve.generated.TunerConstants;
import edu.wpi.first.wpilibj.util.Color;

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

    private final Color BLUE = new Color(0, 0, 255);
    private final Color RED = new Color(255, 0, 0);
    private final Color WHITE = new Color(255, 255, 255);
    private final Color OFF = new Color(0, 0, 0);
    private final Color ORANGE = new Color(255, 15, 0);

    // For sorting
    private boolean sorted;
    private int[] hue = new int[this.buffer.getLength()];

    
    // This is just for radix sort
    // private ArrayList<ArrayList<Integer>> buckets = new ArrayList<ArrayList<Integer>>();
    // private int stage;


    /**
     * Constructor for LED which registers the subsystem and sets a specified portion of the LED lights to the alliance color
     */
    private LED() {
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

    /**
     * Randomizes the hue of each led
     */
    private void randomize()
    {
        for(int i = 0; i < this.buffer.getLength(); i++)
        {
            int hue = (int)(Math.random() * 180);
            this.buffer.setHSV(i, hue, 255, 255);
            this.hue[i] = hue;
        }
        // this.control.setData(buffer);
    }

    /**
     * Sets the led strip to an ordered rainbow
     */
    private void rainbow()
    {
        for(int i = 0; i < this.buffer.getLength(); i++)
        {
            int hue = i*180/this.buffer.getLength();
            this.buffer.setHSV(i, hue, 255, 255);
            this.hue[i] = hue;
        }
        // this.buffer.setHSV(n->n*180/this.buffer.getLength(), n->255, n->255);
        // this.control.setData(buffer);
    }

    /**
     * Shuffles all LEDs
     */
    private void shuffle()
    {
        for(int i = 0; i < this.buffer.getLength(); i++)
        {
            int place = (int)(Math.random() * (this.buffer.getLength() - i)) + i;
            this.swap(i, place);
        }
    }


    private void swap(int index1, int index2)
    {
        Color tcol = this.buffer.getLED(index1);
        this.buffer.setLED(index1, this.buffer.getLED(index2));
        this.buffer.setLED(index2, tcol);
        int thue = this.hue[index1];
        this.hue[index1] = this.hue[index2];
        this.hue[index2] = thue;
    }

    /**
     * Gets a Color object from hsv values
     *
     * @param h the h value [0-180)
     * @param s the s value [0-255]
     * @param v the v value [0-255]
     * @return A color object
     */
    private Color getColor(int h, int s, int v)
    {
        if (s == 0) {
          return new Color(v, v, v);
        }

        // Math stuff
        final int chroma = (s * v) / 255;
        final int region = (h / 30) % 6;
        final int remainder = (int) Math.round((h % 30) * (255 / 30.0));
        final int m = v - chroma;
        final int X = (chroma * remainder) >> 8;
    
        switch (region) {
          case 0:
            return new Color(v, X + m, m);
          case 1:
            return new Color(v - X, v, m);
          case 2:
            return new Color(m, v, X + m);
          case 3:
            return new Color(m, v - X, v);
          case 4:
            return new Color(X + m, m, v);
          default:
            return new Color(v, m, v - X);
        }
    }

    /**
     * Can use any set up as long as it stores hue
     */
    private void bubbleSort()
    {
        for(int i = 0; i < this.buffer.getLength() - 1; i++)
        {
            if(this.hue[i] > this.hue[i+1])
            {
                this.swap(i, i+1);
                return;
            }
        }
        this.sorted = true;
    }

    /**
     * Necessitates that the buffer was set up as a rainbow
     */
    private void insertionSort()
    {
        for(int i = 0; i < this.buffer.getLength(); i++)
        {
            int index = (int)(this.hue[i] / 180.0 * this.buffer.getLength());
            if(i != index)
            {
                this.swap(i, index);
                return;
            }
        }
        this.sorted = true;
    }


    private void mergeSort()
    {
        this.mergeSort(this.hue, 0);
    }

    private void mergeSort(int[] hue, int off)
    {
        if(hue.length <= 1)
            return;
        
        int[] left = new int[hue.length/2];
        Arrays.setAll(left, n->hue[n]);
        int offset = left.length;
        int[] right = new int[hue.length - offset];
        Arrays.setAll(right, n->hue[n + offset]);

        // Check if left side is sorted
        for(int i = 0; i < left.length-1; i++)
        {
            if(left[i] > left[i+1])
            {
                mergeSort(left, off);
                return;
            }
        }

        // Check if right side is sorted
        for(int i = 0; i < right.length-1; i++)
        {
            if(right[i] > right[i+1])
            {
                mergeSort(right, off + offset);
                return;
            }
        }

        // Sort once
        for(int i = 0; i < offset; i++)
        {
            if(hue[i] > hue[offset])
            {
                this.swap(off + i, off + offset);
                return;
            }
        }
        this.sorted = true;
    }

    /* Nah
    private void radixSort()
    {
        if(this.buckets.size() == 0)
        {
            // Create buckets
            for(int i = 0; i < this.buffer.getLength(); i++)
            {
                this.buckets.get(this.hue[i] / (int)(Math.pow(10, this.stage)) % 10).add(i);
            }
        }
        // Bubble sort buckets
        for(ArrayList<Integer> list : this.buckets)
        {
            for(int i = 0; i < list.size(); i++)
            {

            }
        }
        this.stage++;
        this.buckets.clear();
        if(this.stage == 3)
            this.sorted = true;
    }*/

    






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
            this.buffer.setLED(0, this.allianceColorCutoff, RED);
        }
        else {//false is blue
            this.buffer.setLED(0, this.allianceColorCutoff, BLUE);
        }
        this.buffer.setLED(this.allianceColorCutoff, this.driveStateColorCutoff, WHITE);
        this.control.setData(this.buffer);
    }

    public void periodic() {
        if (!this.policeModeEnabled) {
            switch(this.RobotState) { //TODO make the led patterns more intuitive (ie flashing instead of colors)
                case RESTING:
                    this.buffer.setLED(this.allianceColorCutoff, this.driveStateColorCutoff, WHITE);
                    break;
                case NOTE_DETECTED:
                    this.buffer.setRGB(this.allianceColorCutoff, this.driveStateColorCutoff, 115, 81, 226);
                    break;
                case HOMING_NOTE:
                    this.buffer.setRGB(this.allianceColorCutoff, this.driveStateColorCutoff, 255, 183, 3);
                    break;
                case HAS_NOTE:
                    this.buffer.setRGB(this.allianceColorCutoff, this.driveStateColorCutoff, 252, 85, 36);
                    break;
                case HOMING_APRILTAG:
                    this.buffer.setRGB(this.allianceColorCutoff, this.driveStateColorCutoff, 30, 225, 30);
                    break;
            }

            switch(this.swerve.getDriveState()) {
                case FIELD_CENTRIC:
                    this.buffer.setLED(this.driveStateColorCutoff, this.buffer.getLength(), WHITE);
                    break;
                case ROBOT_CENTRIC:
                    this.driveStateControl++;
                    if (this.driveStateControl % 50 == 0) {
                        this.buffer.setLED(this.driveStateColorCutoff, this.buffer.getLength(), WHITE);
                    }
                    else if (this.driveStateControl % 25 == 0) {
                        this.buffer.setLED(this.driveStateColorCutoff, this.buffer.getLength(), OFF);
                    }
                    break;
            }
            this.control.setData(this.buffer);
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
                this.buffer.setLED(0, this.buffer.getLength() / 2, RED);
                this.buffer.setLED(this.buffer.getLength() / 2, this.buffer.getLength(), BLUE);
                control.setData(buffer);
            }
            if (policeMode == 1) {//solid alternating color
                policeModeControl1++;
                if (policeModeControl1 % 50 == 0) {
                    this.buffer.setLED(0, this.buffer.getLength(), RED);
                    control.setData(buffer);
                }
                else if (policeModeControl1 % 25 == 0) {
                    this.buffer.setLED(0, this.buffer.getLength(), BLUE);
                    control.setData(buffer);
                }
            }
            else if (policeMode == 2) {//one flash each color on two halves
                policeModeControl2++;
                if (policeModeControl2 % 40 == 0) {
                    this.buffer.setLED(0, this.buffer.getLength() / 2, RED);
                    this.buffer.setLED(this.buffer.getLength() / 2, this.buffer.getLength(), OFF);
                    control.setData(buffer);
                }
                else if (policeModeControl2 % 20 == 0) {
                    this.buffer.setLED(0, this.buffer.getLength() / 2, OFF);
                    this.buffer.setLED(this.buffer.getLength() / 2, this.buffer.getLength(), BLUE);
                    control.setData(buffer);
                }
            }
            else if (policeMode == 3) {//three flashes each color on two halves
                policeModeControl3++;
                if (policeModeColorControl3 % 2 == 0) {
                    if (policeModeControl3 % 8 == 0) {
                        this.buffer.setLED(0, this.buffer.getLength() / 2, OFF);
                        control.setData(buffer);
                    }

                    else if (policeModeControl3 % 4 == 0) {
                        this.buffer.setLED(0, this.buffer.getLength() / 2, BLUE);
                        control.setData(buffer);


                        if (policeModeControl3 == 20) {
                            policeModeControl3 = 0;
                            policeModeColorControl3++;
                        }
                    }

                    this.buffer.setLED(this.buffer.getLength() / 2, this.buffer.getLength(), OFF);
                    control.setData(buffer);
                }
                else {
                    if (policeModeControl3 % 8 == 0) {
                        this.buffer.setLED(this.buffer.getLength() / 2, this.buffer.getLength(), OFF);
                        control.setData(buffer);
                    }

                    else if (policeModeControl3 % 4 == 0) {
                        this.buffer.setLED(this.buffer.getLength() / 2, this.buffer.getLength(), RED);
                        control.setData(buffer);


                        if (policeModeControl3 == 20) {
                            policeModeControl3 = 0;
                            policeModeColorControl3++;
                        }
                    }

                    this.buffer.setLED(0, this.buffer.getLength() / 2, OFF);
                    control.setData(buffer);
                }
            }
            else if (policeMode == 4) {//segmented flashes

            }
        }
        this.control.setData(buffer);
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