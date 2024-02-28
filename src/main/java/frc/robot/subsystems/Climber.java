package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
    private static Climber instance = null;

    public static final double kMaxSpeed = 0.5;
    public static final double kResetSpeed = -0.2;

    private double maxSpeed = kMaxSpeed;

    private CANSparkMax leftClimber = new CANSparkMax(22, MotorType.kBrushless);
    private CANSparkMax rightClimber = new CANSparkMax(23, MotorType.kBrushless);
    private int springControl = 0;//FILLER VALUE

    private final double ROTATIONS_AT_TOP = 130;
    private final double ROTATIONS_AT_BOTTOM = 0;
    private boolean isStopped = true;


    private Climber(){
        this.setName("Climber");
        this.register();
        this.leftClimber.restoreFactoryDefaults();
        this.rightClimber.restoreFactoryDefaults();
        this.leftClimber.setInverted(false);
        this.rightClimber.setInverted(false);
        this.leftClimber.setIdleMode(IdleMode.kBrake);
        this.rightClimber.setIdleMode(IdleMode.kBrake);
        this.leftClimber.getEncoder().setPosition(0);
        this.rightClimber.getEncoder().setPosition(0);

        SmartDashboard.putNumber("climber speed", kMaxSpeed);
        SmartDashboard.putNumber("reset speed", kResetSpeed);
    }

    public void setSpeed(double speed){
        SmartDashboard.putNumber("raw speed", speed);
        this.leftClimber.set(speed);
        this.rightClimber.set(speed);
        // this.rightClimber.set(speed + springControl);
    }

    public void resetLeftEncoder(){
        this.leftClimber.getEncoder().setPosition(0);
    }

    public void resetRightEncoder(){
        this.rightClimber.getEncoder().setPosition(0);
    }

    public void setLeftSpeed(double speed){
        this.leftClimber.set(speed);
    }

    public void setRightSpeed(double speed){
        this.rightClimber.set(speed);
    }

    public void move(double speed){
        if (speed > 0 && (this.leftClimber.getEncoder().getPosition() > ROTATIONS_AT_TOP || this.rightClimber.getEncoder().getPosition() > ROTATIONS_AT_TOP)) {this.stop(); return;}
        else if (speed < 0 && (this.leftClimber.getEncoder().getPosition() < ROTATIONS_AT_BOTTOM || this.rightClimber.getEncoder().getPosition() < ROTATIONS_AT_BOTTOM)){
            this.stop();
            return;
        }
        this.setSpeed(speed * this.maxSpeed);
        SmartDashboard.putNumber("speed", speed);
    }

    public Command setIdleMode(IdleMode type){
        return new InstantCommand(
            () -> this.leftClimber.setIdleMode(type), this
        );
    }



    

    



    public void stop() {
        this.leftClimber.set(0);
        this.rightClimber.set(0);
    }
    
    public static Climber getInstance(){
        if(Climber.instance == null){
            Climber.instance= new Climber();
        }

        return Climber.instance;
    }
    public void periodic(){
        this.maxSpeed = SmartDashboard.getNumber("climber speed", kMaxSpeed);
        
        SmartDashboard.putNumber("recorded speed", this.leftClimber.get());
        SmartDashboard.putNumber("left encoder", this.leftClimber.getEncoder().getPosition());
        SmartDashboard.putNumber("right encoder", this.rightClimber.getEncoder().getPosition());
    }
}
