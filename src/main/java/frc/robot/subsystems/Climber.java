package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase { 
    
    private static Climber instance = null;

    private CANSparkMax leftClimber = new CANSparkMax(Constants.Climber.LEFT_MOTOR_ID, MotorType.kBrushless);
    private CANSparkMax rightClimber = new CANSparkMax(Constants.Climber.RIGHT_MOTOR_ID, MotorType.kBrushless);

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

    public void goBottom() {
        // TODO: move climber to bottom position
    }

    public static Climber getInstance() {
        if (Climber.instance == null) {
            Climber.instance = new Climber();
        }

        return Climber.instance;
    }
}
