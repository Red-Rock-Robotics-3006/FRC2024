package frc.robot.subsystems.swerve;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public interface AprilTagIO {
    public static final double kStdvX = 5;
    public static final double kStdvY = 5;
    public static final double kStdvTheta = 15000;

    public static final double kStdvScaleDenominator = 30d;

    public static final Pose2d[] tagPoses = null;

    Matrix<N3, N1> getStandardDeviations();
    Pose2d getPoseEstimate();
    double getTimeStamp();
    boolean isValid();
    Field2d getField2d();
    String getName();
}
