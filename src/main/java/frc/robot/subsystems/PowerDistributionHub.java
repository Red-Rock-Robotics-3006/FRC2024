package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PowerDistributionHub extends SubsystemBase{
    private static PowerDistributionHub instance = null;

    private PowerDistribution pdh = new PowerDistribution();

    public static PowerDistributionHub getInstance(){
        if (instance == null) instance = new PowerDistributionHub();

        return instance;
    }

    private PowerDistributionHub(){
        super("pdh");
        pdh.setSwitchableChannel(true);
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("battery voltage", pdh.getVoltage());
    }

    
}
