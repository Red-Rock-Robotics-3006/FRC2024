// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;



import com.revrobotics.CANSparkFlex;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
// import frc.robot.LimelightHelpers;
import frc.robot.Constants.AprilTags;
// import frc.robot.Constants.*;
import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;


import frc.robot.subsystems.swerve.SwerveIO;




public class Shooter extends SubsystemBase {
    



    private static Shooter instance = null;
    private CANSparkFlex leftShooter = new CANSparkFlex(Constants.Shooter.LEFT_MOTOR_ID, MotorType.kBrushless);
    private CANSparkFlex rightShooter = new CANSparkFlex(Constants.Shooter.RIGHT_MOTOR_ID, MotorType.kBrushless);
    
    private CANSparkFlex m_leftAngleMotor = new CANSparkFlex(Constants.Shooter.LEFT_ANGLE_MOTOR_ID, MotorType.kBrushless);
    private CANSparkFlex m_rightAngleMotor = new CANSparkFlex(Constants.Shooter.RIGHT_ANGLE_MOTOR_ID, MotorType.kBrushless);

    private double kP = 0.01; // TODO FILLER
    private double kI = 0.0;
    private double kD = 0.0; // TODO FILLER
    private double kF = 0.2; // TODO FILLER

    private PIDController controller = new PIDController(kP, kI, kD);

    private double[] blueSpeaker = { // All locations field relative
      652.73, // x 16.579342
      218.42, // y 5.547868
      82.90 // z   2.10566
    };

    private double[] redSpeaker = { // All locations field relative
      -1.50, // x  -0.0381
      218.42, // y 5.547868
      82.90 // z   2.10566
    };

    private double[] field = {
        654.23, // X dim 16.617442 : or maybe 651.23? 16.541242
        323.0 // Y dim   8.2042
    };

    /*
    4 652.73 218.42 57.13 180° Blue
    7 -1.50 218.42 57.13 0°    Red
     */
    
    
    public enum Positions {
        SUB_CENTER, SUB_LEFT, SUB_RIGHT, PODIUM_BLUE, PODIUM_RED, AMP
    }
    
    
    // private int sample;
    // private double[] sums;
    // private double[][] data;
    // private String[] samples;
    // private int counter;
    // private int heartbeat;




    private NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
    // private Limelight vision;
    private SwerveIO swerve = new SwerveIO() {
        public double getTargetHeading(){return 0;};
        public void setTargetHeading(double degrees){};
    }; //TunerConstants.DriveTrain; // Swerve object, FILLER
    private Index index = Index.getInstance(); // Index object
      

    private double robotX, robotY, robotYaw; // These should be all the things that we care about, assuming Z is vertical.
    private double targetPitch, targetYaw;
    private boolean seek; // If shooter should seek the speaker
    private boolean autoFire; // If shooter should fire when ready
    private boolean shooting; // Actively shooting
    private boolean ready; // Ready to shoot
    private boolean tagInVision; // If a tag is in vision
    private boolean isOnBlue; // True if on blue alliance, false if on red alliance // TODO get value from smart dashboard or something else

    private boolean snapshot; // For toggling snapshots




    private final double STOW_ANGLE = 0.0; // TODO FILLER
    private final double EXIT_VELOCITY = 0.0; // TODO FILLER





    private Shooter() {
        this.setName("Shooter");
        this.register();
        /* Diagnostics
        this.sums = new double[3];
        this.data = new double[500][3];
        this.sample = 0;
        this.counter = 0;
        this.heartbeat = 0;
        this.seek = true; // REMOVE BEFORE DEPLOYMENT
        this.samples = new String[]{
            "\nOff, Grey, 7ft",
            "\nOff, Grey, 8ft",
            "\nOff, Grey, 9ft",
            "\nOff, Black, 7ft",
            "\nOff, Black, 8ft",
            "\nOff, Black, 9ft",
            "\nOn, Grey, 7ft",
            "\nOn, Grey, 8ft",
            "\nOn, Grey, 9ft",
            "\nOn, Black, 7ft",
            "\nOn, Black, 8ft",
            "\nOn, Black, 9ft"
        };*/

        this.leftShooter.restoreFactoryDefaults();
        this.leftShooter.setInverted(false);
        this.leftShooter.setIdleMode(CANSparkMax.IdleMode.kBrake);

        this.rightShooter.restoreFactoryDefaults();
        this.rightShooter.setInverted(false);
        this.rightShooter.setIdleMode(CANSparkMax.IdleMode.kBrake);

        this.m_rightAngleMotor.restoreFactoryDefaults();
        this.m_rightAngleMotor.setInverted(false);
        this.m_rightAngleMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        this.m_leftAngleMotor.restoreFactoryDefaults();
        this.m_leftAngleMotor.setInverted(false);
        this.m_leftAngleMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    /**
     * Example command factory method.
     *
     * @return a command
     */
    public Command exampleMethodCommand() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(
            () -> {
                /* one-time action goes here */
            });
    }



    public void setHoming(boolean homing)
    {
        this.seek = homing;
        this.autoFire = homing;
    }


    public boolean getHoming()
    {
        return this.seek;
    }


    /**
     * Disables tracking and seeking apriltags and automatically shooting the note and moves to stow angle.
     */
    private void stow()
    {
        this.setHoming(false);
        this.setTarget(this.STOW_ANGLE);
    }


    private void shootNote()
    {
        this.shooting = true;
        this.leftShooter.set(1);
        this.rightShooter.set(1);

        // Spin up
        // while(this.leftShooter.getEncoder().getVelocity() < 0.99)
        //     SmartDashboard.putString("Shooter Status","Spinning up");

        // Await note
        // while(!this.index.hasNote())
        //     SmartDashboard.putString("Shooter Status","Pending note intake");
            
        this.index.startTransfer();

        // Await fire
        // while(this.index.hasNote())
        //     SmartDashboard.putString("Shooter Status","Firing");
        
        // this.index.stopTransfer();

        this.shooting = false;
        this.leftShooter.set(0.3);
        this.rightShooter.set(0.3);

        // Cool off
        // while(!this.shooting && this.leftShooter.getEncoder().getVelocity() > 0.32)
        //     SmartDashboard.putString("Shooter Status","Cooling down");

        SmartDashboard.putString("Shooter Status","Idle");
        
        
        // while()
        // {
        //     SmartDashboard.putNumber("Motor Speed", this.leftShooter.getEncoder().getVelocity());
        // }
        // this.index.startTransfer();

        // while(this.index.hasNote());
        
        // this.leftShooter.set(0.3);
        // this.rightShooter.set(0.3);

        // while(this.leftShooter.getEncoder().getVelocity() > 0.32)
        //     SmartDashboard.putNumber("Motor Speed", this.leftShooter.getEncoder().getVelocity());

        // Move note into flywheels
    }


    /* Deprecated presetShoot(int pos)
    public void presetShoot(int pos)
    {
        switch(pos) // 0 indicates auto speaker, 1x indicates speaker, 2x indicates amp
        {
            case 10: // Pressed up against center subwoofer
                this.setTarget(Constants.Shooter.CENTER_ANGLE);
                break;
            case 11: // Pressed up against left subwoofer
                this.setTarget(Constants.Shooter.LEFT_ANGLE);
                break;
            case 12: // Pressed up against right subwoofer
                this.setTarget(Constants.Shooter.RIGHT_ANGLE);
                break;
            case 13: // Pressed up against blue podium
                this.setTarget(Constants.Shooter.PODIUM_ANGLE, Constants.Shooter.BLUE_PODIUM_HEADING);
                break;
            case 14: // Pressed up against red podium
                this.setTarget(Constants.Shooter.PODIUM_ANGLE, Constants.Shooter.RED_PODIUM_HEADING);
                break;
            case 20: // Pressed up against amp
                this.setTarget(Constants.Shooter.AMP_ANGLE);
                // Make amp bar engage // Amp bar should do this on its own
                break;
            default:
                // Auto seek
        }
        

        this.shootNote();
        if(pos / 10 == 1)
            ; // Make amp bar disengage // Amp bar should do this on its own
    }*/

    // Essential Methods


    @Override
    public void periodic() {
        

        // if(this.seek)
        // {
        //     double speed = this.controller.calculate(this.leftShooter.getEncoder().getPosition(), this.targetPitch) + this.kF;
        //     this.setAngleSpeed(speed);
        // }



        this.tagInVision = this.limelight.getEntry("tv").getDouble(0) > 0;

        // Use pid controller to move the shooter
        this.setAngleSpeed(this.controller.calculate(this.m_rightAngleMotor.getEncoder().getPosition(), this.targetPitch));

        
        if(this.seek && this.tagInVision)
        {
            this.setLocation(this.limelight.getEntry("botpose").getDoubleArray(new double[6]));
            this.track(this.robotX, this.robotY, this.robotYaw, (int)this.limelight.getEntry("tid").getDouble(0));
            /*
            // if(this.sample < this.data.length)
            // {
                
            //     this.data[this.sample][0] = this.robotX;
            //     this.sums[0] += this.robotX;
            //     this.data[this.sample][1] = this.robotY;
            //     this.sums[1] += this.robotY;
            //     this.data[this.sample][2] = this.robotZ;
            //     this.sums[2] += this.robotZ;
            //     this.sample++;
            // }
            // else if(this.sample == this.data.length)
            // {
            //     double meanX = this.sums[0]/(this.data.length);
            //     double meanY = this.sums[1]/(this.data.length);
            //     double meanZ = this.sums[2]/(this.data.length);
            //     double sdX = 0;
            //     double sdY = 0;
            //     double sdZ = 0;
            //     for(double[] point : this.data)
            //     {
            //         sdX += Math.pow(point[0] - meanX, 2);
            //         sdY += Math.pow(point[1] - meanY, 2);
            //         sdZ += Math.pow(point[2] - meanZ, 2);
            //     }
            //     sdX = Math.sqrt(sdX / this.data.length);
            //     sdY = Math.sqrt(sdY / this.data.length);
            //     sdZ = Math.sqrt(sdZ / this.data.length);
            //     System.out.println("\n" +
            //         this.samples[this.counter++] +
            //         "\nμX: " + meanX + " - σX: " + sdX +
            //         "\nμY: " + meanY + " - σY: " + sdY +
            //         "\nμZ: " + meanZ + " - σZ: " + sdZ +
            //         "\nSample: " + this.sample + "\n\n"); 
            //     this.sample++;
            //     this.counter = this.counter % this.samples.length;
            // }
            // else
            // {
            //     if(++this.sample > 600)
            //     {
            //         // System.out.println(this.sample);
            //         this.sums = new double[3];
            //         this.data = new double[500][3];
            //         this.sample = 0;
            //     }
            //     // System.out.println("\n\nX: " + this.sums[0]/(100 - this.sample) + ", Y: " + this.sums[1]/(100 - this.sample) + ", Z: " + this.sums[2]/(100 - this.sample) + ", Sample: " + this.sample + "\n\n");
            //     // this.sample = 100;
            //     // this.sums[0] = 0;
            //     // this.sums[1] = 0;
            //     // this.sums[2] = 0;
            // }
            */
        }
        else if(!this.seek)
        {
            this.stow();
        }

        if(this.seek && this.autoFire && this.ready)
        {
            this.shootNote();
        }


        // POST to smart dashboard periodically
        SmartDashboard.putNumber("RobotX", this.robotX);
        SmartDashboard.putNumber("Robot", this.robotY);
        SmartDashboard.putNumber("RobotYaw", this.robotYaw);
        SmartDashboard.putNumber("TagID", (int)this.limelight.getEntry("tid").getDouble(0));
        SmartDashboard.putBoolean("Tag in Vision", this.tagInVision);
    }





    // Public Interface Methods : Methods that allow outside classes to interface with Shooter


    public void presetShoot(Positions pos)
    {
        switch(pos)
        {
            case SUB_CENTER: // Pressed up against center subwoofer
                this.setTarget(Constants.Shooter.CENTER_ANGLE);
                break;
            case SUB_LEFT: // Pressed up against left subwoofer
                this.setTarget(Constants.Shooter.LEFT_ANGLE);
                break;
            case SUB_RIGHT: // Pressed up against right subwoofer
                this.setTarget(Constants.Shooter.RIGHT_ANGLE);
                break;
            case PODIUM_BLUE: // Pressed up against blue podium
                this.setTarget(Constants.Shooter.PODIUM_ANGLE, Constants.Shooter.BLUE_PODIUM_HEADING);
                break;
            case PODIUM_RED: // Pressed up against red podium
                this.setTarget(Constants.Shooter.PODIUM_ANGLE, Constants.Shooter.RED_PODIUM_HEADING);
                break;
            case AMP: // Pressed up against amp
                this.setTarget(Constants.Shooter.AMP_ANGLE);
                // Make amp bar engage // Amp bar should do this on its own
                break;
        }

        this.shootNote();
    }
    
    /**
     * Takes a snapshot of limelight vision.
     * MUST BE CALLED TWICE PER SNAPSHOT
     */
    public void takeSnapshot()
    {
        if(this.snapshot)
        { // Resets the networktables entry for taking a snapshot, allowing another to be taken.
            this.limelight.getEntry("snapshot").setNumber(0);
            this.snapshot = false;
        }
        else
        { // Takes the snapshot
            this.snapshot = true;
            this.limelight.getEntry("snapshot").setNumber(1);
        }
    }

    /**
     * Manually shoots a note
     * @param on if the shooter is shooting
     */
    public void manualShoot(boolean on)
    {
        if(on)
            this.setShooterSpeed(1);
        else
            this.setShooterSpeed(0.3);
    }

    /**
     * Gives public access to the Shooter subsystem on the robot.
     * @return The Shooter instance
     */
    public static Shooter getInstance()
    {
        if (instance == null) 
            instance = new Shooter();
        return instance;
    }


    // Private Interface Methods : Methods that allow Shooter to interface with fundamental components / set values

    
    /**
     * Calculates the necessary angle of the shooter based on the robot's position and the target's position, both field relative
     * @param botX The field relative x position of the robot
     * @param botY The field relative y position of the robot
     * @param botZ The field relative z position of the robot
     * @param tarX The field relative x position of the target
     * @param tarY The field relative y position of the target
     * @param tarZ The field relative z position of the target
     * @return The required angle of the shooter
     */
    private double calculateAngle(double botX, double botY, double botZ, double tarX, double tarY, double tarZ)
    {
        return this.calculateAngle(botX - tarX, botY - tarY, botZ - tarZ);
    }

    /**
     * Calculates the necessary angle of the shooter based on the robot's position relative to the target
     * @param xDiff The difference in the x values of the robot and target (robotX - targetX)
     * @param yDiff The difference in the Y values of the robot and target (robotY - targetY)
     * @param zDiff The difference in the Z values of the robot and target (robotZ - targetZ)
     * @return The required angle of the shooter
     */
    private double calculateAngle(double xDiff, double yDiff, double zDiff)
    {
        // TODO MATH!
        // this.EXIT_VELOCITY
        return 0;
    }


    /**
     * Sets the stored location of the robot using data from apriltags.
     * @param pose the array containing position and orientation of the robot
     */
    public void setLocation(double[] pose)
    {
        this.setLocation(pose[0], pose[1], pose[5]);
    }

    /**
     * Sets the stored location of the robot.
     * @param botX the robot's field relative x coordinate
     * @param botY the robot's field relative y coordinate
     * @param botYaw the robot's field relative yaw
     */
    public void setLocation(double botX, double botY, double botYaw)
    {
        this.robotX = botX;
        this.robotY = botY;
        this.robotYaw = botYaw;
    }

    /**
     * Sets the robot to track the speaker based on vision
     * @param botX
     * @param botY
     * @param botYaw
     * @param tagID
     */
    private void track(double botX, double botY, double botYaw, int tagID)
    {
        if(tagID < 1 || tagID > 16)
        {
            System.out.println("\u001B[41m\u001B[37mERROR : Shooter.track was called with an invalid tagID!\u001B[0m\n\u001B[41m\u001B[36mDO NOT USE HOMING ON THE SHOOTER UNTIL THIS IS RESOLVED\u001B[0m");
            return;
        }
        // double[] tagPos = Constants.AprilTags.locations[tagID];

        this.setLocation(botX, botY, botYaw);

        // TODO test that math works
        // Calculate heading to point at speaker
        double[] speaker = isOnBlue?this.blueSpeaker:this.redSpeaker;
        double xDiff = botX - speaker[0];
        double yDiff = botY - speaker[1];
        double zDiff = speaker[2];

        double yaw = (Math.toDegrees(Math.atan( yDiff / xDiff )) + 180) % 360;
        this.setTarget(this.calculateAngle(xDiff, yDiff, zDiff), yaw);


        // this.setLocation(this.limelight.getEntry("botpose").getDoubleArray(new double[6]));

        /* Logging
        // System.out.println("Relative Pos:\nX: " + (tagPos[0] - robotX) + ", Y: " + (tagPos[1] - robotY));
        // System.out.println("\nX: " + tagPos[0] + ", Y: " + tagPos[1] + ", Z: " + tagPos[2]);
        // System.out.println("X: " + this.robotX + ", Y: " + this.robotY + ", Z: " + this.robotZ);
        // System.out.println("X: " + tagPos[0]/this.robotX + ", Y: " + tagPos[1]/this.robotY + ", Z: " + tagPos[2]/this.robotZ);
        // Robot position is tag pos + robot pose*/
    }


    /**
     * Sets the target angle of the shooter and target rotation of the robot.
     * @param pitch the target angle of the shooter
     * @param yaw the target field relative rotation of the robot
     */
    private void setTarget(double pitch, double yaw)
    {
        this.setTarget(pitch);
        this.targetYaw = yaw;
        this.swerve.setTargetHeading(this.targetYaw);
    }

    /**
     * Sets the target angle of the shooter.
     * @param pitch the target angle of the shooter
     */
    private void setTarget(double pitch)
    {
        this.targetPitch = pitch;
    }


    private void setAngleSpeed(double speed)
    {
        this.m_rightAngleMotor.set(speed);
        this.m_leftAngleMotor.set(speed);
    }


    private void setShooterSpeed(double speed)
    {
        this.leftShooter.set(speed);
        this.rightShooter.set(speed);
    }
}
