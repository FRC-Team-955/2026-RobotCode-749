package frc.robot.subsystems;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;



import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.util.function.DoubleSupplier;


import com.ctre.phoenix6.hardware.Pigeon2;

public class PoseSubsystem extends SubsystemBase {
    Pigeon2 gyro = new Pigeon2(13, "rio");


    private DifferentialDriveKinematics m_kinematics;
    private  DifferentialDrivePoseEstimator m_poseEstimator;
    private DoubleSupplier l; //link to encoder value? IDK if this works just testing
    private DoubleSupplier r; //object for passing encoder values
    double OldL = 0;
    double OldR = 0;
    double gyroAngle=0;




    //SELF EXPLANATORY......
    public void resetOdometry(Pose2d a){
        m_poseEstimator.resetPose(a);
    }
    //overloaded function for in case a zero-reset is needed
    public void resetOdometry(){

        m_poseEstimator.resetPose(new Pose2d());
    }

    // get pose2d format of best guess pose from the PoseSubsystem
    public Pose2d getPose() {
        return m_poseEstimator.getEstimatedPosition();
    }

    public Rotation2d getGyro(){
        return Rotation2d.kZero; ///////////////////// PLACEHOLDER do a source like the encoders here
    }

    // uh set source?
    public void setSource(DoubleSupplier leftencoder, DoubleSupplier rightencoder){
        l = (()->{ return -leftencoder.getAsDouble()/Constants.DriveConstants.ENCODER_UNITS_PER_METER;});
        r=(()->{ return -rightencoder.getAsDouble()/Constants.DriveConstants.ENCODER_UNITS_PER_METER;});
        OldL = l.getAsDouble();
        OldR=r.getAsDouble();
        m_poseEstimator =
        new DifferentialDrivePoseEstimator(
                m_kinematics,
                new Rotation2d(),
                l.getAsDouble(),
                r.getAsDouble(),
                new Pose2d(),
                VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
                VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));
    }

    //update loop event
    @Override
    public void periodic() {

        // Update the odometry in the periodic block
gyroAngle += Math.atan(((r.getAsDouble()-OldR)-(l.getAsDouble()-OldL))/Constants.DriveConstants.DBASE_WIDTH);
        m_poseEstimator.update(
                gyro.getRotation2d(), l.getAsDouble(), r.getAsDouble()); //use Rotation2d (gyroAngle) for reals
        if (Constants.DEBUG == 1){
            System.out.print("R "); System.out.println(r.getAsDouble());
            System.out.print("L "); System.out.println(l.getAsDouble());
            System.out.print("EPOS: X: "); System.out.print(m_poseEstimator.getEstimatedPosition().getTranslation().getX()); System.out.print(" Y: "); System.out.print(m_poseEstimator.getEstimatedPosition().getTranslation().getY()); System.out.print(" THETA: "); System.out.println(m_poseEstimator.getEstimatedPosition().getRotation().getDegrees());
        }
        SmartDashboard.putNumber("EPOS_X", m_poseEstimator.getEstimatedPosition().getTranslation().getX());
        SmartDashboard.putNumber("EPOS_Y", m_poseEstimator.getEstimatedPosition().getTranslation().getY());
        SmartDashboard.putNumber("EPOS_THETA", m_poseEstimator.getEstimatedPosition().getRotation().getDegrees());
        SmartDashboard.putNumber("EPOS_LE", l.getAsDouble());
        SmartDashboard.putNumber("EPOS_RE", r.getAsDouble());
        SmartDashboard.putNumber("EPOS_dLE", l.getAsDouble()-OldL);
        SmartDashboard.putNumber("EPOS_dRE", r.getAsDouble()-OldR);


        OldL = l.getAsDouble();
        OldR = r.getAsDouble();
    }
}
