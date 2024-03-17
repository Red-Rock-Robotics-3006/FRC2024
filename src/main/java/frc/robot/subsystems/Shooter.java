// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;



import com.revrobotics.CANSparkFlex;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.util.Color;


import frc.robot.subsystems.swerve.SwerveIO;




public class Shooter extends SubsystemBase {
    



    private static Shooter instance = null;
    private CANSparkFlex leftShooter = new CANSparkFlex(Constants.Shooter.LEFT_MOTOR_ID, CANSparkFlex.MotorType.kBrushless);
    private CANSparkFlex rightShooter = new CANSparkFlex(Constants.Shooter.RIGHT_MOTOR_ID, CANSparkFlex.MotorType.kBrushless);
    
    private CANSparkFlex m_leftAngleMotor = new CANSparkFlex(Constants.Shooter.LEFT_ANGLE_MOTOR_ID, CANSparkFlex.MotorType.kBrushless);
    private CANSparkFlex m_rightAngleMotor = new CANSparkFlex(Constants.Shooter.RIGHT_ANGLE_MOTOR_ID, CANSparkFlex.MotorType.kBrushless);

    private DutyCycleEncoder shooterEncoder = new DutyCycleEncoder(9); // TODO Check that this works

    public static double kP = 0.015; // TODO FILLER
    private double kI = 0.0;
    private double kD = 0.0; // TODO FILLER
    public static double kF = 0.046; // TODO FILLER

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



    private NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
    private SwerveIO swerve = new SwerveIO() {
        public double getTargetHeading(){return 0;};
        public void setTargetHeading(double degrees){};
    }; // TODO FILLER
    private Index index = Index.getInstance(); // Index object
      

    private double robotX, robotY; // These should be all the things that we care about, assuming Z is vertical.
    private double targetPitch = 0.45, targetYaw;
    private boolean seek; // If shooter should seek the speaker
    private boolean autoFire; // If shooter should fire when ready
    private boolean shooting; // Actively shooting
    // private boolean locked; // Aimed at speaker with shooter at angle // For use in case of pre-emptive shooting
    private boolean tagInVision; // If a tag is in vision
    private boolean isOnBlue; // True if on blue alliance, false if on red alliance // TODO get value from smart dashboard or something else
    private boolean hasNote; // If the robot has a note

    private boolean snapshot; // For toggling snapshots


    private double[] target;

    public static double encoderTarget = 0;


    private final double STOW_ANGLE = 0.0; // TODO FILLER
    private final double EXIT_VELOCITY = 0.0; // TODO FILLER
    private final double SHOOTER_HEIGHT = 0.0; // TODO FILLER

    private final double CENTER_ANGLE = 0.0; // TODO FILLER
    private final double LEFT_ANGLE = 0.0; // TODO FILLER
    private final double RIGHT_ANGLE = 0.0; // TODO FILLER
    private final double PODIUM_ANGLE = 0.0; // TODO FILLER
    private final double BLUE_PODIUM_HEADING = 0.0; // TODO FILLER
    private final double RED_PODIUM_HEADING = 0.0; // TODO FILLER
    private final double AMP_ANGLE = 0.0; // TODO FILLER





    private Shooter() {
        this.setName("Shooter");
        this.register();

        this.leftShooter.restoreFactoryDefaults();
        this.leftShooter.setInverted(false);
        this.leftShooter.setIdleMode(CANSparkMax.IdleMode.kBrake);

        this.rightShooter.restoreFactoryDefaults();
        this.rightShooter.setInverted(false);
        this.rightShooter.setIdleMode(CANSparkMax.IdleMode.kBrake);

        this.m_rightAngleMotor.restoreFactoryDefaults();
        this.m_rightAngleMotor.setInverted(true);
        this.m_rightAngleMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        this.m_leftAngleMotor.restoreFactoryDefaults();
        this.m_leftAngleMotor.setInverted(false);
        this.m_leftAngleMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        this.setTarget(this.isOnBlue?this.blueSpeaker:this.redSpeaker);
        
        this.controller.setTolerance(0.02); // TODO FILLER

        LEDBuffer leds = new LEDBuffer(58);

        Color blue = new Color(0, 0, 255);
        Color red = new Color(255, 0, 0);

        leds.setLED(0, leds.getLength() / 2, blue);
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






    // Essential Methods


    @Override
    public void periodic() {
        this.controller.setP(kP);
        encoderTarget = SmartDashboard.getNumber("encoder target", 0.7);
        /*
        // Get if the robot can see an AprilTag
        this.tagInVision = this.limelight.getEntry("tv").getDouble(0) > 0;
        
        // Set stored robot location
        if(this.tagInVision)
            this.setLocation(this.limelight.getEntry("botpose").getDoubleArray(new double[6]));
        */
        // Get angle from pos
        double currentPos = this.shooterEncoder.getAbsolutePosition();
        double angle = this.posToDegrees(currentPos); // TODO MATH get angle from pos

        // // Get pos from angle
        // double newPos = 0; // TODO MATH get pos from angle
        
        // Calculate feedForward value.
        double feedForward = kF * Math.cos(Math.toRadians(angle + 30)); // account for gravity: tourque =  r * F * cos(theta) |  r * F is tunable kF term//feedForward.calculate(Math.toRadians(targetAngle), 6, 2);//kF * Math.abs(Math.cos(Math.toRadians(currentAngle))); // account for gravity: tourque =  r * F * cos(theta) |  r * F is tunable kF term
        System.out.println("F: " + feedForward);
        /*
        // If it is set to start homing on the speaker
        if(this.seek)
        {
            this.aim();
            this.swerve.setTargetHeading(this.targetYaw);
        }
        else if(this.shooting)
        {
            this.stow();
        }

        if(this.autoFire && this.isReady() && this.shooting)
        {
            this.fire();
        }

        boolean hadNote = this.index.hasNote();
        if(this.shooting && this.hasNote && !hadNote)
            this.stow();

        this.hasNote = hadNote;
        */
        // Use pid controller to move the shooter
        this.setAngleSpeed(this.controller.calculate(currentPos, encoderTarget) + feedForward);
        // this.setAngleSpeed(feedForward);
        
        System.out.println("Pos: " + currentPos);
        System.out.println("Angle: " + angle);
        kP = SmartDashboard.getNumber("kP", 0.0);
        kF = SmartDashboard.getNumber("kF", 0.046);

        SmartDashboard.putNumber("power", this.m_leftAngleMotor.getAppliedOutput());
        SmartDashboard.putNumber("current encoder", currentPos);
        /*
        // POST to smart dashboard periodically
        SmartDashboard.putNumber("Shooter Angle", this.shooterEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("RobotX", this.robotX);
        SmartDashboard.putNumber("RobotY", this.robotY);
        SmartDashboard.putNumber("TagID", (int)this.limelight.getEntry("tid").getDouble(0));
        SmartDashboard.putBoolean("Tag in Vision", this.tagInVision);
        */
    }





    // Public Interface Methods : Methods that allow outside classes to interface with Shooter


    public boolean getHoming()
    {
        return this.seek;
    }


    public void setHoming(boolean homing)
    {
        this.seek = homing;
    }


    public void setAutoFire(boolean auto)
    {
        this.autoFire = auto;
    }


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
                break;
        }

        // this.aim();
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


    public boolean isAimed()
    {
        return this.controller.atSetpoint();
    }


    public boolean isReady()
    {
        return this.isAimed();
    }

    // Private Interface Methods : Methods that allow Shooter to interface with fundamental components / set values


    private double posToDegrees(double pos)
    {
        return pos*-380.597 + 329.418;
    }


    private double degreesToPos(double degrees)
    {
        return (degrees - 329.418) / -380.597;
    }


    // Shooting

    private void aim()
    {
        // Set values
        if(!shooting)
        {
            SmartDashboard.putString("Shooter Status","Aiming");
            this.shooting = true;
            
            // Spin up shooters
            this.setShooterSpeed(1);
        }

        // Calculate shooter angle
        double xDiff = this.robotX - this.target[0];
        double yDiff = this.robotY - this.target[1];
        double zDiff = this.target[2] - this.SHOOTER_HEIGHT;

        // Calculate robot angle
        double yaw = (Math.toDegrees(Math.atan( yDiff / xDiff )) + 180) % 360;

        // Set Target angles
        this.setTarget(this.calculateAngle(Math.abs(xDiff), Math.abs(yDiff), zDiff), yaw);
    }


    private void fire()
    {
        SmartDashboard.putString("Shooter Status","Firing");
        this.index.startTransfer();
    }

    /**
     * Disables tracking and seeking apriltags and automatically shooting the note and moves to stow angle.
     */
    private void stow()
    {
        SmartDashboard.putString("Shooter Status","Stow");
        this.setHoming(false);
        this.setTarget(this.STOW_ANGLE);
        this.setShooterSpeed(0); // 0.3 for idle
        this.shooting = false;
    }



    // OTHER


    private void setTarget(double[] target)
    {
        this.target = this.target;
    }

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
        double hDist = Math.sqrt(xDiff * xDiff + yDiff + yDiff);
        double vDist = zDiff;

        double theta = Math.atan(hDist - Math.sqrt(hDist * hDist - 2 * 9.8 * hDist * hDist / this.EXIT_VELOCITY * this.EXIT_VELOCITY * (9.8 * hDist * hDist / 2 / this.EXIT_VELOCITY + vDist)) / (9.8 * hDist * hDist / this.EXIT_VELOCITY * this.EXIT_VELOCITY));

        // TODO Check math
        // this.EXIT_VELOCITY
        return theta;
    }

    /**
     * Sets the stored location of the robot using data from apriltags.
     * @param pose the array containing position and orientation of the robot
     */
    public void setLocation(double[] pose)
    {
        this.setLocation(pose[0], pose[1]);
    }

    /**
     * Sets the stored location of the robot.
     * @param botX the robot's field relative x coordinate
     * @param botY the robot's field relative y coordinate
     * @param botYaw the robot's field relative yaw
     */
    public void setLocation(double botX, double botY)
    {
        this.robotX = botX;
        this.robotY = botY;
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
    }

    /**
     * Sets the target angle of the shooter.
     * @param pitch the target angle of the shooter
     */
    private void setTarget(double pitch)
    {
        // Convert degrees to encoder
        double targetDegrees = this.degreesToPos(pitch);

        this.targetPitch = targetDegrees;
    }


    public void setAngleSpeed(double speed)
    {
        System.out.println(speed);
        this.m_rightAngleMotor.set(speed);
        this.m_leftAngleMotor.set(speed);
    }


    private void setShooterSpeed(double speed)
    {
        this.leftShooter.set(speed);
        this.rightShooter.set(speed);
    }
}
