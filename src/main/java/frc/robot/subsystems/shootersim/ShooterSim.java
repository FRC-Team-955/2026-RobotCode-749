package frc.robot.subsystems.shootersim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

import java.util.ArrayList;

import static frc.robot.RobotState.GLOBAL_POSE;

public class ShooterSim {


    InterpolationTable DistanceToAngV = new InterpolationTable();

    public ShooterSim(){
        DistanceToAngV.add(0,0);
        DistanceToAngV.add(2,28);
        DistanceToAngV.add(4,59);
    }

    public double getShooterVel(Pose2d robotPose, Pose3d target){
        double cV = DistanceToAngV.getLinear(Math.sqrt(Math.pow(target.getX()- robotPose.getX(),2)+Math.pow(target.getY()- robotPose.getY(),2)));
        if (cV > 20 && cV<66){
            return cV;
        }
        return -1;
    }

    public Rotation2d toFaceHub(){
        //new Pose3d(4.62017, 4.03450, 1.8288, new Rotation3d()); //Center of target
        double x = 4.62017 - GLOBAL_POSE.getX();
        double y  = 4.0345 - GLOBAL_POSE.getY();
        return new Rotation2d(Math.atan2(y,x)+Math.PI);
    }
}
