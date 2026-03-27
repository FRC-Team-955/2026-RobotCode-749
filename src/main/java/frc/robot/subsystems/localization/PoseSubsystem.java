package frc.robot.subsystems.localization;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;



import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.util.function.DoubleSupplier;


import com.ctre.phoenix6.hardware.Pigeon2;

import static frc.robot.RobotState.*;

@Logged
public class PoseSubsystem extends SubsystemBase {
    Pigeon2 gyro = new Pigeon2(Constants.DriveConstants.PIGEON_ID, "rio");

    private LimeLightIO limelight1;



    DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Constants.DriveConstants.TRACK_WIDTH); //27 = drivebase width?????
    private  DifferentialDrivePoseEstimator poseEstimator = new DifferentialDrivePoseEstimator(kinematics,gyro.getRotation2d(),0,0,new Pose2d());
    private DoubleSupplier l; //link to encoder value
    private DoubleSupplier r; //object for passing encoder values
    double OldL = 0;
    double OldR = 0;




    public PoseSubsystem() {
        poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.28,0.28,Math.PI/6)); // yaw angle +; xy are ok
        limelight1 = new LimeLightIO("",-0.179,0.0558+0.2732,0.097/2+ 0.302,90.0,-14.0,90.0-30.75); //from center of robot. IDK if rotations chain or abs

    }







    //SELF EXPLANATORY
    public void resetOdometry(Pose2d a){
        poseEstimator.resetPose(a);
        globalPose = a;
    }

    public void resetGyro() {
        gyro.reset();
    }

    // get pose2d format of best guess pose from the PoseSubsystem
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }


    // uh set source?
    public void setSource(DoubleSupplier leftencoder, DoubleSupplier rightencoder){
        l = (()->{ return -leftencoder.getAsDouble();});
        r=(()->{ return -rightencoder.getAsDouble();});
        OldL = l.getAsDouble();
        OldR=r.getAsDouble();
        poseEstimator =
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


    boolean hadTarget = false;


    //update loop event
    @Override
    public void periodic() {


            // Update the odometry in the periodic block
            if (limelight1.hasTarget()) { //IF TARGET
                if (!hadTarget) {
                    hadTarget = true;
                    System.out.println("Limelight target acquired!");
                }
                poseEstimator.addVisionMeasurement(limelight1.getEstimate(), limelight1.getTimestamp());
            } else {
                if (hadTarget) {
                    System.out.println("NO LIMELIGHT TARGET");
                    hadTarget = false;
                }
            }
            llpublisher.set(limelight1.getEstimate());


        if (l == null || r == null) return; // NPE
        poseEstimator.update(
                gyro.getRotation2d(), l.getAsDouble(), r.getAsDouble()); //use Rotation2d (gyroAngle) for reals
        if (Constants.DEBUG == 1){
            System.out.print("R "); System.out.println(r.getAsDouble());
            System.out.print("L "); System.out.println(l.getAsDouble());
            System.out.print("EPOS: X: "); System.out.print(poseEstimator.getEstimatedPosition().getTranslation().getX()); System.out.print(" Y: "); System.out.print(poseEstimator.getEstimatedPosition().getTranslation().getY()); System.out.print(" THETA: "); System.out.println(poseEstimator.getEstimatedPosition().getRotation().getDegrees());
        }
        SmartDashboard.putNumber("EPOS_X", poseEstimator.getEstimatedPosition().getTranslation().getX());
        SmartDashboard.putNumber("EPOS_Y", poseEstimator.getEstimatedPosition().getTranslation().getY());
        SmartDashboard.putNumber("EPOS_THETA", poseEstimator.getEstimatedPosition().getRotation().getDegrees());
        SmartDashboard.putNumber("EPOS_LE", l.getAsDouble());
        SmartDashboard.putNumber("EPOS_RE", r.getAsDouble());
        SmartDashboard.putNumber("EPOS_dLE", l.getAsDouble()-OldL);
        SmartDashboard.putNumber("EPOS_dRE", r.getAsDouble()-OldR);

        globalPose = poseEstimator.getEstimatedPosition();
        publisher.set(globalPose);

        OldL = l.getAsDouble();
        OldR = r.getAsDouble();


    }
}
