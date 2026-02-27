package frc.robot.subsystems;
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
import frc.robot.DSAndFieldUtil;
import frc.robot.LimelightHelpers;

import static frc.robot.DSAndFieldUtil.*;

public class PoseSubsystem extends SubsystemBase {
    Pigeon2 gyro = new Pigeon2(Constants.DriveConstants.PIGEON_ID, "rio");



    DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Constants.DriveConstants.DBASE_WIDTH); //27 = drivebase width?????
    private  DifferentialDrivePoseEstimator m_poseEstimator = new DifferentialDrivePoseEstimator(kinematics,gyro.getRotation2d(),0,0,new Pose2d());
    private DoubleSupplier l; //link to encoder value
    private DoubleSupplier r; //object for passing encoder values
    double OldL = 0;
    double OldR = 0;


    // Basic targeting data
   double tx = LimelightHelpers.getTX("");  // Horizontal offset from crosshair to target in degrees
    double ty = LimelightHelpers.getTY("");  // Vertical offset from crosshair to target in degrees
    double ta = LimelightHelpers.getTA("");  // Target area (0% to 100% of image)
    boolean hasTarget = LimelightHelpers.getTV(""); // Do you have a valid target?

    double txnc = LimelightHelpers.getTXNC("");  // Horizontal offset from principal pixel/point to target in degrees
    double tync = LimelightHelpers.getTYNC("");  // Vertical offset from principal pixel/point to target in degrees


    public PoseSubsystem() {
        // Switch to pipeline 0
        LimelightHelpers.setPipelineIndex("", 0);
        // Set a custom crop window for improved performance (-1 to 1 for each value)
        LimelightHelpers.setCropWindow("", -0.5, 0.5, -0.5, 0.5);

// Change the camera pose relative to robot center (x forward, y left, z up, degrees)
        LimelightHelpers.setCameraPose_RobotSpace("",
                0.5,    // Forward offset (meters)
                0.0,    // Side offset (meters)
                0.5,    // Height offset (meters)
                0.0,    // Roll (degrees)
                30.0,   // Pitch (degrees)
                0.0     // Yaw (degrees)
        );

// Set AprilTag offset tracking point (meters)
        LimelightHelpers.setFiducial3DOffset("",
                0.0,    // Forward offset
                0.0,    // Side offset
                0.5     // Height offset
        );

// Configure AprilTag detection
        LimelightHelpers.SetFiducialIDFiltersOverride("", new int[]{1, 2, 3, 4}); // Only track these tag IDs
        LimelightHelpers.SetFiducialDownscalingOverride("", 2.0f); // Process at half resolution

// Adjust keystone crop window (-0.95 to 0.95 for both horizontal and vertical)
        LimelightHelpers.setKeystone("", 0.1, -0.05);
    }







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


    // uh set source?
    public void setSource(DoubleSupplier leftencoder, DoubleSupplier rightencoder){
        l = (()->{ return -leftencoder.getAsDouble()/Constants.DriveConstants.ENCODER_UNITS_PER_METER;});
        r=(()->{ return -rightencoder.getAsDouble()/Constants.DriveConstants.ENCODER_UNITS_PER_METER;});
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

    //update loop event
    @Override
    public void periodic() {
        LimelightHelpers.PoseEstimate limelightMeasurement;
        if (DSAndFieldUtil.isTestMode() || DSAndFieldUtil.isRedAlliance()){
             limelightMeasurement= LimelightHelpers.getBotPoseEstimate_wpiRed("");
        }
        else{
            limelightMeasurement= LimelightHelpers.getBotPoseEstimate_wpiBlue("");
        }
        // Update the odometry in the periodic block
        m_poseEstimator.addVisionMeasurement(limelightMeasurement.pose, limelightMeasurement.timestampSeconds);
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

        if(!isSim()) {
            publisher.set(m_poseEstimator.getEstimatedPosition());
            GLOBAL_POSE = m_poseEstimator.getEstimatedPosition();

        }
        if(isSim()) {
            double vel = ((r.getAsDouble()-OldR)+l.getAsDouble()-OldL)/0.04; //TEST THISSSSS
            DSAndFieldUtil.ROBOT_VX = vel*Math.cos(DSAndFieldUtil.GLOBAL_POSE.getRotation().getRadians());
            DSAndFieldUtil.ROBOT_VY = vel*Math.sin(DSAndFieldUtil.GLOBAL_POSE.getRotation().getRadians());
        }


    }
}
