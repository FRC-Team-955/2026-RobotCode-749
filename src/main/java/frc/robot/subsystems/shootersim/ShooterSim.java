package frc.robot.subsystems.shootersim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

import java.util.ArrayList;

import static frc.robot.RobotState.GLOBAL_POSE;

public class ShooterSim {


    InterpolationTable DistanceToAngV = new InterpolationTable();
    double MAX_OUTPUT = 67; // this might actually be a good number.......
    double MIN_OUTPUT = 20;

    public ShooterSim(){
        /// TODO: These need tuning. inPt is in meters, between center of robot and center of target, horizontally only.
        /// Starting at hub center auto position, drive the robot such that it is a known distance from the target. Then hold driver controller y
        /// Find or add a line below that corresponds to the distance from the robot center to target
        /// Tune outPt higher if robot undershoots
        /// Tune lower if overshoots
        /// Also change such that the table is zero if the robot is too close to the target
        /// Finally, check MAX and MIN output to make sure they fit table

        DistanceToAngV.add(0,0); /// Do not change
        DistanceToAngV.add(1,0); /// change this one's inPt to the furthest point (within a few meters) the robot CANNOT shoot from
        DistanceToAngV.add(1.1,24); /// change this to be a little higher inPt than the one directly above, with well tuned outPt
        DistanceToAngV.add(2,28);
        DistanceToAngV.add(3,45);
        DistanceToAngV.add(4,59);
        DistanceToAngV.add(5,64);
    }

    public double getShooterVel(Pose2d robotPose, Pose3d target){
        double cV = DistanceToAngV.getLinear(Math.sqrt(Math.pow(target.getX()- robotPose.getX(),2)+Math.pow(target.getY()- robotPose.getY(),2)));
        if (cV > MIN_OUTPUT && cV<MAX_OUTPUT){
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
