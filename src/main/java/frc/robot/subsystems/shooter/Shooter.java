// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;

import java.sql.Driver;

import com.revrobotics.CANSparkFlex;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Settings;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.index.Index;
import frc.robot.subsystems.swerve.SwerveIO;
import frc.robot.subsystems.swerve.generated.TunerConstants;




public class Shooter extends SubsystemBase {
    
    private boolean pitchHoming = true;


    private static Shooter instance = null;
    private CANSparkFlex topShooter = new CANSparkFlex(Constants.Shooter.TOP_MOTOR_ID, CANSparkFlex.MotorType.kBrushless);
    private CANSparkFlex bottomShooter = new CANSparkFlex(Constants.Shooter.BOTTOM_MOTOR_ID, CANSparkFlex.MotorType.kBrushless);
    
    private CANSparkFlex m_leftAngleMotor = new CANSparkFlex(Constants.Shooter.LEFT_ANGLE_MOTOR_ID, CANSparkFlex.MotorType.kBrushless);
    private CANSparkFlex m_rightAngleMotor = new CANSparkFlex(Constants.Shooter.RIGHT_ANGLE_MOTOR_ID, CANSparkFlex.MotorType.kBrushless);

    private DutyCycleEncoder shooterEncoder = new DutyCycleEncoder(0); // TODO Check that this works

    public static double kP = 0.0; // NOT THE REAL VALUE // TODO FILLER
    private double kI = 0.0;
    private double kD = 0.0; // TODO FILLER
    public static double kF = 0.0; // TODO FILLER

    public static final double kFinalP = 6; //-5.5
    public static final double kFinalF = 0; //0.048

    private PIDController controller = new PIDController(kP, kI, kD);

    private double[] blueSpeaker = { // All locations field relative
      -0.0381, // -1.50, // x  -0.0381
      5.39, // 218.42, // y 5.547868
      2.10566 // 82.90 // z   2.10566
    };

    private double[] redSpeaker = { // All locations field relative
      16.579342, // 652.73, // x 16.579342
      5.547868, // 218.42, // y 5.547868
      2.10566 // 82.90 // z   2.10566
    };

    /*
    4 652.73 218.42 57.13 180° Blue
    7 -1.50 218.42 57.13 0°    Red
     */
    
    
    public enum Positions {
        SUB_CENTER, SUB_LEFT, SUB_RIGHT, PODIUM_BLUE, PODIUM_RED, AMP, AUTO_SIDES, INTAKE
    }



    // private NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight-intake");
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
    private double horizontalDistance;
    private boolean isInRange;

    private boolean snapshot; // For toggling snapshots


    private double[] target;

    public static double encoderTarget = 0.8;


    private final double STOW_ANGLE = 37; // TODO FILLER
    private double EXIT_VELOCITY = 7.0; // m/s // TODO FILLER
    private final double SHOOTER_HEIGHT = 0.13; // Meters // 20 Inches // TODO FILLER
    private double DISTANCE_FEED = -0.2;
    private final double RANGE = 4;

    private final double CENTER_ANGLE = 0.0; // TODO FILLER
    private final double LEFT_ANGLE = 0.0; // TODO FILLER
    private final double RIGHT_ANGLE = 0.0; // TODO FILLER
    private final double PODIUM_ANGLE = 0.0; // TODO FILLER
    private final double BLUE_PODIUM_HEADING = 0.0; // TODO FILLER
    private final double RED_PODIUM_HEADING = 0.0; // TODO FILLER
    private final double AMP_ANGLE = 0.0; // TODO FILLER


    private final double LOW_ENCODER = 0.82; // 20  degrees
    private final double HIGH_ENCODER = 0.72; // 54 degrees


    public static final double kTopShooterAmpSpeed = 0.15;
    public static final double kBottomShooterAmpSpeed = 0.25;
    public static final double kAmpAngle = 49;


    private Shooter() {
        this.setName("Shooter");
        this.register();

        this.topShooter.restoreFactoryDefaults();
        this.topShooter.setInverted(false);
        this.topShooter.setIdleMode(CANSparkMax.IdleMode.kBrake);
        this.topShooter.setSmartCurrentLimit(Constants.Shooter.SHOOT_CURRENT_LIMIT);
        this.topShooter.burnFlash();

        this.bottomShooter.restoreFactoryDefaults();
        this.bottomShooter.setInverted(false);
        this.bottomShooter.setIdleMode(CANSparkMax.IdleMode.kBrake);
        this.bottomShooter.setSmartCurrentLimit(Constants.Shooter.SHOOT_CURRENT_LIMIT);
        this.bottomShooter.burnFlash();

        this.m_rightAngleMotor.restoreFactoryDefaults();
        this.m_rightAngleMotor.setInverted(false);
        this.m_rightAngleMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        this.m_leftAngleMotor.restoreFactoryDefaults();
        this.m_leftAngleMotor.setInverted(true);
        this.m_leftAngleMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        // if (DriverStation.getAlliance().isPresent())
        // this.isOnBlue = DriverStation.getAlliance().get() == Alliance.Blue;
        // else this.isOnBlue = true;
        this.isOnBlue = !DriverStation.getAlliance().isPresent() || DriverStation.getAlliance().get() == Alliance.Blue;

        this.setTarget(this.isOnBlue?this.blueSpeaker:this.redSpeaker);
        
        this.controller.setTolerance(0.02); // TODO FILLER

        this.targetPitch = STOW_ANGLE;

        SmartDashboard.putBoolean("On Blue", this.isOnBlue);

        SmartDashboard.putNumber("Exit Velocity", this.EXIT_VELOCITY);
        SmartDashboard.putNumber("Distance Feed", this.DISTANCE_FEED);

        SmartDashboard.putNumber("top shooter amp speed", kTopShooterAmpSpeed);
        SmartDashboard.putNumber("bottom shooter amp speed", kBottomShooterAmpSpeed);

    } 






    // Essential Methods


    @Override
    public void periodic() {
        // Get robot pos from Localization
        this.updateLocation();


        this.EXIT_VELOCITY = SmartDashboard.getNumber("Exit Velocity", 8);
        kP = SmartDashboard.getNumber("kP", kFinalP);
        kF = SmartDashboard.getNumber("kF", kFinalF);
        this.controller.setP(kP);

        this.DISTANCE_FEED = SmartDashboard.getNumber("Distance Feed", this.DISTANCE_FEED);
        


        this.setTarget();
        /* Replaced by this.setTarget()
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
        if(!Constants.Settings.SHOOTER_HOMING_ENABLED || !this.seek)
            targetAngle = this.targetPitch;
        this.setTarget(targetAngle, yaw);
        */
        // else
            // this.setTarget(STOW_ANGLE);

        this.aim();
        /* Replaced by this.aim()
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
            swerve.setTargetHeading(this.targetYaw);*/
        

        SmartDashboard.putNumber("TargetX", this.target[0]);
        SmartDashboard.putNumber("TargetY", this.target[1]);

        SmartDashboard.putNumber("left v", topShooter.getEncoder().getVelocity());
        SmartDashboard.putNumber("right v", bottomShooter.getEncoder().getVelocity());

        // SmartDashboard.putNumber("top shooter amp speed", kTopShooterAmpSpeed);
        // SmartDashboard.putNumber("bottom shooter amp speed", kBottomShooterAmpSpeed);
        // SmartDashboard.putNumber("amp angle", kAmpAngle);
    }






    // Public Interface Methods : Methods that allow outside classes to interface with Shooter

    public void runAmpShot() {
        this.topShooter.set(SmartDashboard.getNumber("top shooter amp speed", kTopShooterAmpSpeed));
        this.bottomShooter.set(SmartDashboard.getNumber("bottom shooter amp speed", kBottomShooterAmpSpeed));
    }

    public void runAmpAngle() {
        this.setTarget(SmartDashboard.getNumber("amp angle", kAmpAngle));
    }

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
        System.out.println("homing is " + homing);
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
            case INTAKE:
                this.setTarget(STOW_ANGLE);
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
        Localization.takeSnapshot();
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
        return this.isAimed() && this.inRange();
    }


    public boolean inRange()
    {
        return this.isInRange;
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

    private void setTarget()
    {
        double xDiff = this.robotX - this.target[0];
        double yDiff = this.robotY - this.target[1];
        double zDiff = this.target[2] - this.SHOOTER_HEIGHT;

        
        // this.horizontalDistance = Math.sqrt(xDiff * xDiff + yDiff * yDiff);
        // SmartDashboard.putNumber("HDist", this.horizontalDistance);
        // Set values
        if(Constants.Settings.SHOOTER_HOMING_ENABLED && this.seek)
        {
            // Only auto aim if the robot is in range
            if(this.isInRange)
            {
                // Calculate robot angle
                double yaw = (Math.toDegrees(Math.atan( yDiff / xDiff )) - this.homingOffset) % 360; //  + (isOnBlue?0:180)


                double targetAngle = this.calculateAngle(Math.abs(xDiff), Math.abs(yDiff), zDiff);
                SmartDashboard.putNumber("Calculated Encoder", this.degreesToPos(targetAngle));
                SmartDashboard.putNumber("Calculated Angle", targetAngle);
                SmartDashboard.putNumber("Target Heading", yaw);
                SmartDashboard.putNumber("Homing Offset", this.homingOffset);
                this.setTarget(targetAngle, yaw);
            }
            else
                this.setTarget(STOW_ANGLE); // TODO Change to this.targetPitch
        }
    }


    private void aim()
    {
        // Get angle from pos
        double ePos = this.shooterEncoder.getAbsolutePosition();
        double angle = this.posToDegrees(ePos);

        // In the case of an extreme target value, prevent the shooter from being told to move there
        double eTarget = Math.max(HIGH_ENCODER, Math.min(LOW_ENCODER, this.degreesToPos(this.targetPitch)));
        
        // Calculate feedForward value.
        double feedForward = kF * Math.cos(Math.toRadians(angle + 30));
        
        // Use pid controller to move the shooter
        double speed = this.controller.calculate(ePos, eTarget) + feedForward;
        this.setAngleSpeed(speed);

        if(Settings.SHOOTER_HOMING_ENABLED && this.seek && this.isInRange)
            this.swerve.setTargetHeading(this.targetYaw);

            
        SmartDashboard.putNumber("encoder target", eTarget);
        SmartDashboard.putNumber("angle target", this.targetPitch);
        SmartDashboard.putNumber("current encoder", ePos);
        SmartDashboard.putNumber("current angle", angle);

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
        this.presetShoot(Positions.INTAKE);
        this.setShooterSpeed(0); // 0.3 for idle
        this.shooting = false;
    }



    // OTHER


    private void setTarget(double[] target)
    {
        this.target = target;
    }

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
        SmartDashboard.putNumber("Vertical Distance", vDist);

        /* Too fancy
        double tof = Math.sqrt(hDist * hDist + vDist * vDist)/EXIT_VELOCITY;
        double drop = (1.49352) * tof * tof;
        SmartDashboard.putNumber("ToF", tof);
        SmartDashboard.putNumber("Drop", drop);
        double theta = Math.toDegrees(Math.atan((vDist + drop) / hDist));*/
        double theta = Math.toDegrees(Math.atan(vDist / hDist)) + hDist * DISTANCE_FEED;

        // TODO Check math
        return theta;
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
        SmartDashboard.putNumber("shooter pitch speed", speed);
        this.m_rightAngleMotor.set(speed); // TODO FIX
        this.m_leftAngleMotor.set(speed); 
    }


    public void setShooterSpeed(double speed)
    {
        this.topShooter.set(speed);
        this.bottomShooter.set(speed);
    }

    public void setAmpSpeed(double speed)
    {
        this.topShooter.set(speed);
        this.bottomShooter.set(speed);
    }


    private void updateLocation()
    {
        Pose2d pose = Localization.getPose();
        this.tagInVision = Localization.tagInVision();
        this.robotX = pose.getX();
        this.robotY = pose.getY();
        double h = Math.abs(this.robotX - this.target[0]);
        double y = Math.abs(this.robotY - this.target[1]);
        this.horizontalDistance = Math.sqrt(h*h + y*y);
        this.isInRange = this.horizontalDistance < RANGE;
        SmartDashboard.putNumber("Horizontal Distance", this.horizontalDistance);
        SmartDashboard.putBoolean("In Range", this.inRange());
    }
}