package frc.robot.subsystems;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants;

import java.util.function.DoubleSupplier;

public class PoseSubsystem {


    private final DifferentialDriveOdometry odo;
    private DoubleSupplier l; //link to encoder value? IDK if this works just testing
    private DoubleSupplier r; //object for passing encoder values
    public PoseSubsystem(){
        odo = new DifferentialDriveOdometry(Rotation2d.kZero, 0,0);
    }

        //SELF EXPLANATORY......
    public void resetOdometry(Pose2d a){
        odo.resetPose(a);
    }
    //overloaded function for in case a zero-reset is needed
    public void resetOdometry(){
        odo.resetPose(new Pose2d(0,0,Rotation2d.kZero));
    }

    // get pose2d format of best guess pose from the PoseSubsystem
    public Pose2d getPose() {
        return odo.getPoseMeters();
    }

    public Rotation2d getGyro(){
        return Rotation2d.kZero; ///////////////////// PLACEHOLDER do a source like the encoders here
    }

    // uh set source?
    public void setSource(DoubleSupplier leftencoder, DoubleSupplier rightencoder){
        l = leftencoder;
        r=rightencoder;
    }

    //update loop event
    public void periodic() {
        // Update the odometry in the periodic block
        odo.update(
            getGyro(),
                l.getAsDouble(),
                r.getAsDouble()
        );

        if (Constants.DEBUG == 1){
            System.out.print("R "); System.out.println(r.getAsDouble());
            System.out.print("L "); System.out.println(l.getAsDouble());
        }
    }
}




























