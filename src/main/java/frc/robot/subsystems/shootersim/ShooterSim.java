package frc.robot.subsystems.shootersim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

import java.util.ArrayList;

public class ShooterSim {
    private double SHOOTER_X = 0.5; //offset from middle of robot
    private double SHOOTER_Y = 0;
    private double SHOOTER_Z = 0.4;
    private double SHOOTER_LAUNCH_THETA = 2*Math.PI/5; //Radians from horiz!
    InterpolationTable Table = new InterpolationTable(); // this thing is meant to track angular velocity vs landing point!
    public ShooterSim(){
        Table.add(0,0);
        Table.add(60,15); //meters
    }


    public ArrayList<Pose3d> SimShot(double voltage, Pose2d robotPose, double robot_vx, double robot_vy){
        ///  yup I did the math https://artofproblemsolving.com/texer/fofuwvgw
        double dt = 0.031415; // I felt like it, OK?!
        double g = 9.81;
        ArrayList<Pose3d> Poses = new ArrayList<Pose3d>();

        double alpha = robotPose.getRotation().getRadians();


        double shooterOffsetX = SHOOTER_X * Math.cos(alpha) - SHOOTER_Y * Math.sin(alpha);
        double shooterOffsetY = SHOOTER_X * Math.sin(alpha) + SHOOTER_Y * Math.cos(alpha);

// add to robot position
        double x = robotPose.getX() + shooterOffsetX;
        double y = robotPose.getY() + shooterOffsetY;
        double h = SHOOTER_Z;
        double theta = SHOOTER_LAUNCH_THETA;


        // Voltage to horizontal range along shot
        double b = x + Table.getLinear(voltage); // b = x + range along shot direction

        // Precompute launch velocity from derived formula
        double dx = b - x;
        double v = Math.sqrt((g * dx * dx) / (2 * Math.pow(Math.cos(theta), 2) * (h + dx * Math.tan(theta))));

        double vh = v * Math.cos(theta);
        double vz = v * Math.sin(theta);

        double cosDir = Math.cos(alpha + Math.PI);
        double sinDir = Math.sin(alpha + Math.PI);

        // Maximum time based on dx
        double tMax = dx / (vh);

        // Simulation loop
        for (double t = 0.0; t <= tMax; t += dt) {

            double X = x + (vh * cosDir + robot_vx) * t;
            double Y = y + (vh * sinDir + robot_vy) * t;
            double Z = h + vz * t - 0.5 * g * t * t;

            if (Z < 0) break; // stop when hitting the floor

            Poses.add(new Pose3d(X, Y, Z, new Rotation3d()));
        }
return Poses;
    }
}
