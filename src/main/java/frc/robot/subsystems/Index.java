// package frc.robot.subsystems;

// import com.revrobotics.CANSparkMax;

// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;


// public class Index extends SubsystemBase{

//     private final CANSparkMax m_topMotor = new CANSparkMax(Constants.Index.TOP_MOTOR_ID, CANSparkMax.MotorType.kBrushless);
//     private final CANSparkMax m_bottomMotor = new CANSparkMax(Constants.Index.BOTTOM_MOTOR_ID, CANSparkMax.MotorType.kBrushless);
//     private final DigitalInput beamBrake = new DigitalInput(Constants.Index.SWITCH_CHANNEL_ID);

//     private static Index instance = null;

//     private Index() {
//         this.setName("Index");
//         this.register();

//         this.m_topMotor.restoreFactoryDefaults();
//         this.m_topMotor.setInverted(false);
//         this.m_topMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

//         this.m_bottomMotor.restoreFactoryDefaults();
//         this.m_bottomMotor.setInverted(true);
//         this.m_bottomMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
//     }

//     public void setSpeed(double speed) {
//         this.m_topMotor.set(speed);
//         this.m_bottomMotor.set(speed);
//     }

//     public void startTransfer() {
//         this.setSpeed(0.1);//test this when possible
//     }

//     public void stopTransfer() {
//         this.setSpeed(0);
//     }

//     public boolean hasNote() {
//         if (this.beamBrake.get()) return true;
//         return false;
//     }

//     /**
//      * Singleton architecture which returns the singular instance of LED
//      * @return the instance (which is instantiated when first called)
//      */
//     public static Index getInstance(){
//         if (instance == null) instance = new Index();
//         return instance;
//     }
// }