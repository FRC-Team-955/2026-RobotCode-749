package frc.robot.subsystems.localization;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.LimelightHelpers;
import frc.robot.RobotState;


// Sketchy higher-level limelight IO process. Threads pose reads off to a worker
public final class LimeLightIO {
    private String llname;
    private Thread Worker;
    private volatile Pose2d estimate = new Pose2d();
    private volatile double tiemstamp;
    private volatile boolean target;

    LimeLightIO(String name, double fw, double side, double height, double roll, double pitch, double yaw) {
        llname = name;
        LimelightHelpers.setPipelineIndex(llname, 0); // Default Pipeline

// Change the camera pose relative to robot center (x forward, y left, z up, degrees)
        LimelightHelpers.setCameraPose_RobotSpace(llname,
                fw,    // Forward offset (meters)
                side,    // Side offset (meters)
                height,    // Height offset (meters)
                roll,    // Roll (degrees)
                pitch,   // Pitch (degrees)
                yaw     // Yaw (degrees)
        );
        LimelightHelpers.setLEDMode_ForceOn(llname);

// Set AprilTag offset tracking point (meters)
        LimelightHelpers.setFiducial3DOffset(llname,
                0.0,    // Forward offset
                0.0,    // Side offset
                0.0     // Height offset
        );


        LimelightHelpers.SetFiducialDownscalingOverride(llname, 2.0f); // Process at half resolution

// Adjust keystone crop window (-0.95 to 0.95 for both horizontal and vertical)
        LimelightHelpers.setKeystone(llname, 0.1, -0.05);


        Worker = LimeLightIOWorker();
        Worker.setDaemon(true); // i think this makes it clean up?
        Worker.start();
    }
    public Thread LimeLightIOWorker(){
        return new Thread(()->{
            while (!Thread.interrupted()) {
                LimelightHelpers.PoseEstimate est;
                if (RobotState.isTestMode() || RobotState.isRedAlliance()) {
                    est = LimelightHelpers.getBotPoseEstimate_wpiRed(llname);
                } else {
                    est = LimelightHelpers.getBotPoseEstimate_wpiBlue(llname);
                }

                if(est == null) {
                    estimate = new Pose2d();
                    target = false;
                    // timestamp?
                }
                else {
                    estimate = est.pose;
                    tiemstamp = est.timestampSeconds;
                    target = est.tagCount >0; // yay saved a LLHelpers call here
                }
                try {
                    Thread.sleep(10); // busy wait?? might as well spam poll since its slow
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
            }

        });
    }
    public Pose2d getEstimate() {return estimate;}
    public boolean hasTarget() {return target;}
    public double getTimestamp() {return tiemstamp;}
}
