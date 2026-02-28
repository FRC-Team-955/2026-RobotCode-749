package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;


// Just a class to contain annoying functions we dont wanna put everywhere related to game state!
// Also for stuff to be put on DS, to be accessd all over in the code
public class RobotState {


    public static Pose2d addPose(Pose2d a, Pose2d b){
        return new Pose2d(a.getX()+b.getX(), a.getY()+b.getY(), new Rotation2d(a.getRotation().getRadians()+b.getRotation().getRadians()));
    }
    public static Pose2d INITIAL_POSE = new Pose2d(3.6,4,new Rotation2d());
    public static Pose2d GLOBAL_POSE =INITIAL_POSE;
    public static double ROBOT_VX=0;
    public static double ROBOT_VY = 0;

    public static boolean isRedAlliance() {
        return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue).equals(DriverStation.Alliance.Red);
    }

    public static boolean isTeleop(){
        return DriverStation.isTeleopEnabled();
    }

    public static boolean isTestMode(){
        return DriverStation.isTestEnabled();
    }

    public static boolean isSim(){
        return RobotBase.isSimulation();
    }


}
