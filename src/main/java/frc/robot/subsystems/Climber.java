package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
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

    // CHANGE THIS TO NUMBER OF MOTOR ROTATIONS FOR CLIMBER TO BE AT TOP
    private double rotationsAtTop = 5;

    private PIDController controller = new PIDController(kP, kI, kD);

    private Climber() {
        this.setName("Climber");
        this.register();

        this.leftClimber.restoreFactoryDefaults();
        this.rightClimber.restoreFactoryDefaults();

        this.leftClimber.setInverted(false);
        this.rightClimber.setInverted(true);

        this.leftClimber.setIdleMode(IdleMode.kBrake);
        this.rightClimber.setIdleMode(IdleMode.kBrake);

        this.goBottom();
        this.goBottom();
    }

    public void freeze() {
        this.leftClimber.set(0);
        this.rightClimber.set(0);
    }

    public void setPower(double output) throws Exception {
        if (output < 0 || output > 1.0) {
            throw new Exception("Power parameter needs to be in-between 0.0 and 1.0");
        }

        this.leftClimber.set(output);
        this.rightClimber.set(output);
    }

    public void goTop() {
        // TODO: move climber to top position
        
    }

    public void goBottom() throws Exception {
        // get current climber position
        double leftPos = this.leftClimber.getEncoder().getPosition();
        double rightPos = this.rightClimber.getEncoder().getPosition();

        // goal position
        double setPoint = 0.0;

        if (Math.abs(leftPos-rightPos) > 0.00000001) {
            // climbers are at different positions
            throw new Exception("Climber arms are in different places");
        }

        this.controller.setSetpoint(setPoint);
        // use pid to move climber to bottom position
        while (!this.controller.atSetpoint()) {
            this.setPower(this.controller.calculate(this.leftClimber.getEncoder().getPosition()));
        }
    }

    public static Climber getInstance() {
        if (Climber.instance == null) {
            Climber.instance = new Climber();
        }

        return Climber.instance;
    }
}
