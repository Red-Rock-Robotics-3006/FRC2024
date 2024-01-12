package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Transfer extends SubsystemBase{
    private final CANSparkMax m_transferMotor = new CANSparkMax(Constants.Transfer.TRANSFER_MOTOR_ID, CANSparkMax.MotorType.kBrushless);

    private static Transfer instance = null;

    private Transfer() {
        this.setName("Transfer");
        this.register();
    }
}
