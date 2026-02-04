package frc.robot.subsystems;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants;

import java.util.function.DoubleSupplier;

public class PoseSubsystem {

    private Pose2d BestGuess;
    private final DifferentialDriveOdometry odo;
    private DoubleSupplier l;
    private DoubleSupplier r; //object for passing encoder values
    public PoseSubsystem(){
        odo = new DifferentialDriveOdometry(Rotation2d.kZero, 0,0);
    }


    public void resetOdometry(Pose2d a){
        odo.resetPose(a);
    }

    public Pose2d getPose() {
        return odo.getPoseMeters();
    }

    public void setSource(DoubleSupplier leftencoder, DoubleSupplier rightencoder){
        l = leftencoder;
        r=rightencoder;
    }


    public void periodic() {
        // Update the odometry in the periodic block
        odo.update(
            Rotation2d.kZero,
                l.getAsDouble(),
                r.getAsDouble()
        );

        if (Constants.DEBUG == 1){
            System.out.print("R "); System.out.println(r.getAsDouble());
            System.out.print("L "); System.out.println(l.getAsDouble());
        }
    }
}
