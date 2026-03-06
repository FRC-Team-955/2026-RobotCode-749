package frc.robot.subsystems;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;



import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.util.function.DoubleSupplier;


import com.ctre.phoenix6.hardware.Pigeon2;
import frc.robot.RobotState;


import static frc.robot.RobotState.*;

public class PoseSubsystem extends SubsystemBase {
    Pigeon2 gyro = new Pigeon2(Constants.DriveConstants.PIGEON_ID, "rio");



    DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Constants.DriveConstants.DBASE_WIDTH); //27 = drivebase width?????
    private  DifferentialDrivePoseEstimator m_poseEstimator = new DifferentialDrivePoseEstimator(kinematics,gyro.getRotation2d(),0,0,new Pose2d());
    private DoubleSupplier l; //link to encoder value
    private DoubleSupplier r; //object for passing encoder values
    double OldL = 0;
    double OldR = 0;








    //SELF EXPLANATORY...... but kinda no works
    public void resetOdometry(Pose2d a){
        m_poseEstimator.resetPose(a);
        GLOBAL_POSE = a;
    }


    // get pose2d format of best guess pose from the PoseSubsystem
    public Pose2d getPose() {
        return m_poseEstimator.getEstimatedPosition();
    }


    // uh set source?
    public void setSource(DoubleSupplier leftencoder, DoubleSupplier rightencoder){
        l = (()->{ return -leftencoder.getAsDouble();});
        r=(()->{ return -rightencoder.getAsDouble();});
        OldL = l.getAsDouble();
        OldR=r.getAsDouble();
        m_poseEstimator =
        new DifferentialDrivePoseEstimator(
                kinematics,
                gyro.getRotation2d(),
                l.getAsDouble(),
                r.getAsDouble(),
                new Pose2d(),
                VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(3)), //gyro accuracy
                VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10)));
    }


    StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault()
            .getStructTopic("Pose", Pose2d.struct).publish();

    StructPublisher<Pose2d> llpublisher = NetworkTableInstance.getDefault()
            .getStructTopic("LimelightPose", Pose2d.struct).publish();


    //update loop event
    @Override
    public void periodic() {
        double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
        Pose2d llpose = new Pose2d();
        double[] lldata;
        lldata = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose").getDoubleArray(new double[6]);

        // Update the odometry in the periodic block
        if(tv==1) { //IF TARGET
            //pose
            llpose = new Pose2d(lldata[0],lldata[1],Rotation2d.fromDegrees(lldata[5]));
            if(RobotState.isRedAlliance()){// limelight is the only thing in this whole code that is not symmetric depending on alliance!
                llpose = new Pose2d(16-llpose.getX(), 8-llpose.getY(),new Rotation2d(llpose.getRotation().getRadians()+Math.PI));
            }
            m_poseEstimator.addVisionMeasurement(llpose, Timer.getFPGATimestamp());
            String llpipeline = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tdclass").getString("NA");
            System.out.print("LIMELIGHT ("); System.out.print(llpipeline); System.out.print(") POSE: [X:"); System.out.print(llpose.getX()); System.out.print(", Y:");System.out.print(llpose.getY());System.out.print(", THETA:");System.out.print(llpose.getRotation().getRadians());System.out.println("]");

        }
        else{
            System.out.println("NO LIMELIGHT TARGET");
        }
        llpublisher.set(llpose);
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






            GLOBAL_POSE = m_poseEstimator.getEstimatedPosition();
        publisher.set(GLOBAL_POSE);




        OldL = l.getAsDouble();
        OldR = r.getAsDouble();








    }
}
