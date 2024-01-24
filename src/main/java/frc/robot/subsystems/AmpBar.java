package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AmpBar extends SubsystemBase{
    private static AmpBar instance;

    private CANSparkMax ampBarMotor = new CANSparkMax(39010, MotorType.kBrushless); // REPLACE FILLER MOTOR ID

    private AmpBar() {
        this.setName("Amp Bar");
        this.register();

        this.ampBarMotor.restoreFactoryDefaults();

        this.ampBarMotor.setInverted(false);

        this.ampBarMotor.setIdleMode(IdleMode.kBrake);

        this.goRest();
    }

    public void freeze() {
        this.ampBarMotor.set(0);
    }

    private void setPower(double s) {
        this.ampBarMotor.set(s);
    }

    public void goRest() {

    }

    public static AmpBar getInstance() {
        if (AmpBar.instance == null) {
            AmpBar.instance = new AmpBar();
        }

        return AmpBar.instance;
    }

}
