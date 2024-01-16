package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase { 
    
    // change IDS

    private CANSparkMax leftClimber = new CANSparkMax(0, MotorType.kBrushless);
    private CANSparkMax rightClimber = new CANSparkMax(1, MotorType.kBrushless);

    // TODO: keep track of where the climber is

    public Climber() {
        this.leftClimber.restoreFactoryDefaults();
        this.rightClimber.restoreFactoryDefaults();

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
        this.rightClimber.set(-output);
    }

    public void setVert() {
        // TODO: move climber to a certain vertical point
    }

}
