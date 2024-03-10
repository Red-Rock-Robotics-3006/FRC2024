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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;


import frc.robot.subsystems.swerve.SwerveIO;
import frc.robot.subsystems.swerve.generated.TunerConstants;




public class Shooter extends SubsystemBase {
    
    private boolean pitchHoming = false;


    private static Shooter instance = null;
    private CANSparkFlex leftShooter = new CANSparkFlex(Constants.Shooter.LEFT_MOTOR_ID, CANSparkFlex.MotorType.kBrushless);
    private CANSparkFlex rightShooter = new CANSparkFlex(Constants.Shooter.RIGHT_MOTOR_ID, CANSparkFlex.MotorType.kBrushless);
    
    // private CANSparkFlex m_leftAngleMotor = new CANSparkFlex(Constants.Shooter.LEFT_ANGLE_MOTOR_ID, CANSparkFlex.MotorType.kBrushless);
    private CANSparkFlex m_rightAngleMotor = new CANSparkFlex(Constants.Shooter.RIGHT_ANGLE_MOTOR_ID, CANSparkFlex.MotorType.kBrushless);

    private DutyCycleEncoder shooterEncoder = new DutyCycleEncoder(0); // TODO Check that this works

    public static double kP = 0.025; // NOT THE REAL VALUE // TODO FILLER
    private double kI = 0.0;
    private double kD = 0.0; // TODO FILLER
    public static double kF = 0.025; // TODO FILLER

    public static final double kFinalP = 5.5;
    public static final double kFinalF = 0.048;

    private PIDController controller = new PIDController(kP, kI, kD);

    private double[] blueSpeaker = { // All locations field relative
      -0.0381, // -1.50, // x  -0.0381
      5.547868, // 218.42, // y 5.547868
      2.10566 // 82.90 // z   2.10566
    };

    private double[] redSpeaker = { // All locations field relative
      16.579342, // 652.73, // x 16.579342
      5.547868, // 218.42, // y 5.547868
      2.10566 // 82.90 // z   2.10566
    };

    private double[] field = {
        16.617442, // 654.23, // X dim 16.617442 : or maybe 651.23? 16.541242
        8.2042 // 323.0 // Y dim   8.2042
    };

    /*
    4 652.73 218.42 57.13 180° Blue
    7 -1.50 218.42 57.13 0°    Red
     */
    
    
    public enum Positions {
        SUB_CENTER, SUB_LEFT, SUB_RIGHT, PODIUM_BLUE, PODIUM_RED, AMP, AUTO_SIDES
    }



    private NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight-shooter");
    private SwerveIO swerve = TunerConstants.DriveTrain;
    private Index index = Index.getInstance(); // Index object
      

    private double robotX = this.redSpeaker[0]/2, robotY = this.redSpeaker[1]; // These should be all the things that we care about, assuming Z is vertical.
    private double targetPitch = 45, targetYaw;
    private boolean seek; // If shooter should seek the speaker
    private boolean autoFire; // If shooter should fire when ready
    private boolean shooting; // Actively shooting
    // private boolean locked; // Aimed at speaker with shooter at angle // For use in case of pre-emptive shooting
    private boolean tagInVision; // If a tag is in vision
    private boolean isOnBlue; // True if on blue alliance, false if on red alliance // TODO get value from smart dashboard or something else
    private boolean hasNote; // If the robot has a note
    private double homingOffset;

    private boolean snapshot; // For toggling snapshots


    private double[] target;

    public static double encoderTarget = 0.8;


    private final double STOW_ANGLE = 45 ; // TODO FILLER
    private double EXIT_VELOCITY = 4.0; // m/s // TODO FILLER
    private final double SHOOTER_HEIGHT = 0.508; // Meters // 20 Inches // TODO FILLER

    private final double CENTER_ANGLE = 0.0; // TODO FILLER
    private final double LEFT_ANGLE = 0.0; // TODO FILLER
    private final double RIGHT_ANGLE = 0.0; // TODO FILLER
    private final double PODIUM_ANGLE = 0.0; // TODO FILLER
    private final double BLUE_PODIUM_HEADING = 0.0; // TODO FILLER
    private final double RED_PODIUM_HEADING = 0.0; // TODO FILLER
    private final double AMP_ANGLE = 0.0; // TODO FILLER


    private final double LOW_ENCODER = 0.78; // 32.3 degrees
    private final double HIGH_ENCODER = 0.69; // 67.5 degrees



    private Shooter() {
        this.setName("Shooter");
        this.register();

        this.leftShooter.restoreFactoryDefaults();
        this.leftShooter.setInverted(true);
        this.leftShooter.setIdleMode(CANSparkMax.IdleMode.kBrake);
        this.leftShooter.setSmartCurrentLimit(Constants.Shooter.SHOOT_CURRENT_LIMIT);
        this.leftShooter.burnFlash();

        this.rightShooter.restoreFactoryDefaults();
        this.rightShooter.setInverted(false);
        this.rightShooter.setIdleMode(CANSparkMax.IdleMode.kBrake);
        this.rightShooter.setSmartCurrentLimit(Constants.Shooter.SHOOT_CURRENT_LIMIT);
        this.rightShooter.burnFlash();

        this.m_rightAngleMotor.restoreFactoryDefaults();
        this.m_rightAngleMotor.setInverted(true);
        this.m_rightAngleMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        // this.m_leftAngleMotor.restoreFactoryDefaults();
        // this.m_leftAngleMotor.setInverted(false);
        // this.m_leftAngleMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        this.isOnBlue = false;

        this.setTarget(this.isOnBlue?this.blueSpeaker:this.redSpeaker);
        
        this.controller.setTolerance(0.02); // TODO FILLER

        this.targetPitch = STOW_ANGLE;

        SmartDashboard.putNumber("Exit Velocity", this.EXIT_VELOCITY);

        
    } 






    // Essential Methods


    @Override
    public void periodic() {
        // System.out.println(this.robotX);
        this.EXIT_VELOCITY = SmartDashboard.getNumber("Exit Velocity", 8);
        this.controller.setP(kP);
        
        // Get if the robot can see an AprilTag
        this.tagInVision =  this.limelight.getEntry("tv").getDouble(0) > 0;
        
        // Set stored robot location
        if(this.tagInVision)
            this.setLocation(this.limelight.getEntry("botpose_wpiblue").getDoubleArray(new double[6]));
        
        SmartDashboard.putBoolean("Tag in Vision", this.tagInVision);
        SmartDashboard.putNumber("Bot x", this.robotX);
        SmartDashboard.putNumber("Bot y", this.robotY);
        
        // double targetAngle = this.calculateAngle(this.robotX, this.robotY, SHOOTER_HEIGHT);
        // SmartDashboard.putNumber("Calculated Angle", targetAngle);




        // Calculate shooter angle
        double xDiff = this.robotX - this.target[0];
        double yDiff = this.robotY - this.target[1];
        double zDiff = this.target[2] - this.SHOOTER_HEIGHT;

        // Calculate robot angle // TODO TEST MATH
        double yaw = (Math.toDegrees(Math.atan( yDiff / xDiff )) - this.homingOffset) % 360; //  + (isOnBlue?0:180)

        double targetAngle = this.calculateAngle(Math.abs(xDiff), Math.abs(yDiff), zDiff);
        SmartDashboard.putNumber("Calculated Encoder", this.degreesToPos(targetAngle));
        SmartDashboard.putNumber("Calculated Angle", targetAngle);
        SmartDashboard.putNumber("Target Heading", yaw);
        SmartDashboard.putNumber("Homing Offset", this.homingOffset);
        // this.setTarget(targetAngle);
        // Set Target angles
        if(!Constants.Settings.SHOOTER_PITCH_HOMING_ENABLED || !this.seek)
            targetAngle = this.targetPitch;
        this.setTarget(targetAngle, yaw);
        // else
            // this.setTarget(STOW_ANGLE);



        // Get angle from pos
        double currentPos = this.shooterEncoder.getAbsolutePosition();
        double angle = this.posToDegrees(currentPos);

        // In the case of an extreme target value, prevent the shooter from being told to move there
        double eTarget = Math.max(HIGH_ENCODER, Math.min(LOW_ENCODER, this.degreesToPos(this.targetPitch)));
        
        // Calculate feedForward value.
        double feedForward = kF * Math.cos(Math.toRadians(angle + 30));
        
        // Use pid controller to move the shooter
        // double speed = this.controller.calculate(currentPos, eTarget) + feedForward;
        double speed = this.controller.calculate(currentPos, eTarget) + feedForward;
        // System.out.println(this.posToDegrees(0.8) + " : " + this.posToDegrees(0.7));
        // System.out.println(this.targetPitch + " : " + eTarget);
        this.setAngleSpeed(speed);
        if(this.seek)
            swerve.setTargetHeading(this.targetYaw);
        
        kP = SmartDashboard.getNumber("kP", kFinalP);
        kF = SmartDashboard.getNumber("kF", kFinalF);

        SmartDashboard.putNumber("encoder target", eTarget);
        SmartDashboard.putNumber("shooter target", this.targetPitch);
        // SmartDashboard.putNumber("power", this.m_leftAngleMotor.getAppliedOutput());
        SmartDashboard.putNumber("current encoder", currentPos);
        SmartDashboard.putNumber("current angle", angle);
        SmartDashboard.putNumber("TargetX", this.target[0]);
        SmartDashboard.putNumber("TargetY", this.target[1]);

        SmartDashboard.putNumber("left v", leftShooter.getEncoder().getVelocity());
        SmartDashboard.putNumber("right v", rightShooter.getEncoder().getVelocity());
    }





    // Public Interface Methods : Methods that allow outside classes to interface with Shooter
    public double getOffset(){
        return this.homingOffset;
    }

    public void setOffset(double offset)
    {
        this.homingOffset = offset;
    }


    public boolean getHoming()
    {
        return this.seek;
    }


    public void setHoming(boolean homing)
    {
        this.seek = homing;
        if(homing)
            SmartDashboard.putString("Shooter Status", "Homing");
    }


    public void setAutoFire(boolean auto)
    {
        this.autoFire = auto;
    }

    /**
     * Sets the shooter to a preset angle based on the position of the robot as provided
     * SUB_CENTER = 45 degrees
     * SUB_LEFT = 60 degrees
     * @param pos the position of the robot
     */
    public void presetShoot(Positions pos)
    {
        this.setHoming(false);
        switch(pos)
        {
            case SUB_CENTER: // Pressed up against center subwoofer
                this.setTarget(37);
                break;
            case SUB_LEFT: // Pressed up against left subwoofer
                this.setTarget(57);
                break;
            case SUB_RIGHT: // Pressed up against right subwoofer
                this.setTarget(47);
                break;
            case PODIUM_BLUE: // Pressed up against blue podium
                this.setTarget(0, 0);
                break;
            case PODIUM_RED: // Pressed up against red podium
                this.setTarget(0, 0);
                break;
            case AMP: // Pressed up against amp
                this.setTarget(0);
                break;
            case AUTO_SIDES:
                this.setTarget(60);
                break;    
        }
        SmartDashboard.putString("Shooter Status", "Preset");
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
            this.setShooterSpeed(0); // 0.3 for idle
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
    public void stow()
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
        this.target = target;
    }

    // /**
    //  * Calculates the necessary angle of the shooter based on the robot's position and the target's position, both field relative
    //  * @param botX The field relative x position of the robot in meters
    //  * @param botY The field relative y position of the robot in meters
    //  * @param botZ The field relative z position of the robot in meters
    //  * @return The required angle of the shooter
    //  */
    // private double calculateAngle(double botX, double botY, double botZ)
    // {
    //     return this.calculateAngle(Math.abs(botX - this.target[0]), Math.abs(botY - this.target[1]), Math.abs(botZ - this.target[2]));
    // }

    /**
     * Calculates the necessary angle of the shooter based on the robot's position relative to the target
     * @param xDiff The difference in the x values of the robot and target (robotX - targetX) in meters
     * @param yDiff The difference in the Y values of the robot and target (robotY - targetY) in meters
     * @param zDiff The difference in the Z values of the robot and target (robotZ - targetZ) in meters
     * @return The required angle of the shooter
     */
    private double calculateAngle(double xDiff, double yDiff, double zDiff)
    {
        double hDist = Math.sqrt(xDiff * xDiff + yDiff * yDiff);
        double vDist = zDiff;
        
        // Equation to solve for the angle that the shooter needs to be at.
        // DOES NOT WORK
        // double theta = Math.atan(hDist - Math.sqrt(hDist * hDist - 2 * 9.8 * hDist * hDist / this.EXIT_VELOCITY * this.EXIT_VELOCITY * (9.8 * hDist * hDist / 2 / this.EXIT_VELOCITY + vDist)) / (9.8 * hDist * hDist / this.EXIT_VELOCITY * this.EXIT_VELOCITY));
        
        // Equation to estimate the angle that the shooter needs to be at.
        SmartDashboard.putNumber("Horizontal Distance", hDist);
        SmartDashboard.putNumber("Vertical Distance", vDist);
        double tof = Math.sqrt(hDist * hDist + vDist * vDist)/EXIT_VELOCITY;
        double drop = (1.49352) * tof * tof;
        SmartDashboard.putNumber("ToF", tof);
        SmartDashboard.putNumber("Drop", drop);
        double theta = Math.toDegrees(Math.atan((vDist + drop) / hDist));

        // TODO Check math
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
        // SmartDashboard.putNumber("TrueX", botX);
        // SmartDashboard.putNumber("TrueY", botY);
        // SmartDashboard.putNumber("OffsetX", this.field[0]/2);
        // SmartDashboard.putNumber("OffsetY", this.field[1]/2);
        this.robotX = botX; // - this.field[0]/2;
        this.robotY = botY; // - this.field[1]/2;
    }


    /**
     * Sets the target angle of the shooter and target rotation of the robot.
     * @param pitch the target angle of the shooter in degrees
     * @param yaw the target field relative rotation of the robot
     */
    private void setTarget(double pitch, double yaw)
    {

        this.setTarget(pitch);
        this.targetYaw = yaw;
    }

    /**
     * Sets the target angle of the shooter.
     * @param pitch the target angle of the shooter in degrees
     */
    public void setTarget(double pitch)
    {

        // Prevent target angle from being set to an extreme value
        double max = this.posToDegrees(HIGH_ENCODER);
        double min = this.posToDegrees(LOW_ENCODER);
        this.targetPitch = Math.max(min, Math.min(max, pitch));
    }
    

    public void setAngleSpeed(double speed)
    {
        // System.out.println(speed);
        // this.m_rightAngleMotor.set(speed); // TODO FIX
        // this.m_leftAngleMotor.set(speed);
    }


    public void setShooterSpeed(double speed)
    {
        this.leftShooter.set(speed);
        this.rightShooter.set(speed);
    }
}
