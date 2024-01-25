package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase { 
    
    private static Climber instance = null;

    private CANSparkMax leftClimber = new CANSparkMax(Constants.Climber.LEFT_MOTOR_ID, MotorType.kBrushless);
    private CANSparkMax rightClimber = new CANSparkMax(Constants.Climber.RIGHT_MOTOR_ID, MotorType.kBrushless);


    // CHANGE PID NUMBERS
    // I only need kP, kI and kD can stay at 0

    private double kP = 0.0000000001;
    private double kI = 0;
    private double kD = 0;

    private double targetAngle = 0.0;

    // CHANGE THIS TO NUMBER OF MOTOR DEGREES FOR CLIMBER TO BE AT TOP
    private double topAngle = 150;
    private double bottomAngle = 0;

    private double currentAngle = 0;

    private PIDController controller = new PIDController(kP, kI, kD);
    private RelativeEncoder encoder = leftClimber.getEncoder();

    private Climber() {
        this.setName("Climber");
        this.register();

        this.leftClimber.restoreFactoryDefaults();
        this.rightClimber.restoreFactoryDefaults();

        this.leftClimber.setInverted(false);
        this.rightClimber.setInverted(true);

        this.leftClimber.setIdleMode(IdleMode.kBrake);
        this.rightClimber.setIdleMode(IdleMode.kBrake);
    }

    /**
     * Stops all motors, likely redundant because motors on brake, but a backup
     */
    public void freeze() {
        this.setPower(0);
    }

    /**
     * Sets the power of both climber motors
     * @param output a double from -1.0 to 1.0, the percentage power for the motors
     */
    private void setPower(double output) {
        this.leftClimber.set(output);
        this.rightClimber.set(output);
    }

     /**
     * Get angle of motor currently
     * @return current angle of motor
     */
    public double getAngle() {
        double angle;
        double currentTick = encoder.getPosition();
        angle = 6.0645 * (currentTick + 95.63);
        return angle;
    }

    /**
     * Converts angle to ticks
     * @param angle angle to convert to ticks
     * @return ticks from given angle
     */
    public double toTicks(double angle) {
        double ticks = (angle / 6.0645) - 95.63;
        return ticks;
    }

    /**
     * Sets the target of the motors to be the lowest elevation possible
     */
    public void goBottom() {
        this.targetAngle = this.bottomAngle;
    }

    /**
     * Sets the target of the motors to be the highest elevation possible
     */
    public void goTop() {
        this.targetAngle = this.topAngle;
    }

    /**
     * @return  a Climber object, that encapsulates the logic to move the climber
     * @throws Exception
     */
    public static Climber getInstance() {
        if (Climber.instance == null) {
            Climber.instance = new Climber();
        }

        return Climber.instance;
    }

    @Override
    public void periodic() {
        this.controller.setPID(kP, kI, kD);
        this.currentAngle = this.getAngle();
        this.setPower(controller.calculate(this.leftClimber.getEncoder().getPosition(), this.targetAngle));
    }
}
