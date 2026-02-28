package frc.robot.subsystems.shootersim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

import java.util.ArrayList;

import static frc.robot.RobotState.GLOBAL_POSE;

public class ShooterSim {
    private double SHOOTER_X = 0.22; //meters offset from middle of robot
    private double SHOOTER_Y = 0;
    private double SHOOTER_Z = 0.49; //19.3 inches

    InterpolationTable AngularVelocityToLandingPt = new InterpolationTable(); // track angular velocity -> landing point!
    InterpolationTable AngularVelocityLaunchAngle = new InterpolationTable(); // track angular velocity -> launch angle
    public ShooterSim(){
        AngularVelocityToLandingPt.add(0,0);
        AngularVelocityToLandingPt.add(40,2.72); //107 in
        AngularVelocityToLandingPt.add(58,4.06); //meters!  160 in
        AngularVelocityToLandingPt.add(70,4.66); // THIS ISNT TUNED!

        AngularVelocityLaunchAngle.add(0,Math.PI/2);
        AngularVelocityLaunchAngle.add(40,Math.PI/2-Math.toRadians(30)); //
        AngularVelocityLaunchAngle.add(58,Math.PI/2-Math.toRadians(24)); //23/25 deg off vertical
    }


    public ArrayList<Pose3d> SimShot(double angV, Pose2d robotPose, double robot_vx, double robot_vy){
        ///  yup I did the math https://artofproblemsolving.com/texer/fofuwvgw
        double dt = 0.02; // 
        double g = 9.81; //m/s^2
        ArrayList<Pose3d> Poses = new ArrayList<Pose3d>();

        double alpha = robotPose.getRotation().getRadians();


        double shooterOffsetX = SHOOTER_X * Math.cos(alpha) - SHOOTER_Y * Math.sin(alpha);
        double shooterOffsetY = SHOOTER_X * Math.sin(alpha) + SHOOTER_Y * Math.cos(alpha);

// add to robot position
        double x = robotPose.getX() + shooterOffsetX;
        double y = robotPose.getY() + shooterOffsetY;
        double h = SHOOTER_Z;
        double theta = AngularVelocityLaunchAngle.getPoly(angV);


        // Voltage to horizontal range along shot
        double b = x + AngularVelocityToLandingPt.getPoly(angV); // b = x + range along shot direction

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


    // ONLY WORKS FOR FLAT POLYGONS WITH CONSTANT Z COORD
    public boolean IsHit(ArrayList<Pose3d> shot, ArrayList<Pose3d> targetPolygon) {

        if (shot.size() < 2 || targetPolygon.size() < 3) //Arrays too small to be trajectory or a polygon
            return false;

        double planeZ = targetPolygon.get(0).getZ(); //
        boolean upwards = true;

        for (int i = 0; i < shot.size() - 1; i++) {

            Pose3d p0 = shot.get(i);
            Pose3d p1 = shot.get(i + 1);

            double z0 = p0.getZ();
            double z1 = p1.getZ();


            // Check if segment crosses the plane but ONLY if down through
            if (!(z0 > planeZ && z1 < planeZ))
                upwards = true;
            else{
                upwards = false;
            }




            double t = (planeZ - z0) / (z1 - z0);

            if (t < 0 || t > 1)
                continue;

            double x = p0.getX() + t * (p1.getX() - p0.getX());
            double y = p0.getY() + t * (p1.getY() - p0.getY());

            if (pointInPolygon2D(x, y, targetPolygon)) {
                return !upwards;
            }
        }

        return false;
    }


    // yeah i converted some sketch implementation from my raytracer hope it works
    private boolean pointInPolygon2D(double x, double y, ArrayList<Pose3d> polygon) {

        boolean inside = false; //toggle counter every edge cross
        int size = polygon.size(); // number of verts

        for (int i = 0, j = size - 1; i < size; j = i++) { //cycle every edge

            double xi = polygon.get(i).getX();
            double yi = polygon.get(i).getY();
            double xj = polygon.get(j).getX();
            double yj = polygon.get(j).getY();

            boolean intersect = ((yi > y) != (yj > y)) &&
                    (x < (xj - xi) * (y - yi) / (yj - yi) + xi); // reimplement from FlatRTI.cpp

            if (intersect)
                inside = !inside;
        }

        return inside; //even = false, odd = true
    }


    // Gives radians to set current angle to, to face hub exactly
    public Rotation2d toFaceHub(){
        //new Pose3d(4.62017, 4.03450, 1.8288, new Rotation3d()); //Center of target
        double x = 4.62017 - GLOBAL_POSE.getX();
        double y  = 4.0345 - GLOBAL_POSE.getY();
        return new Rotation2d(Math.atan2(y,x)+Math.PI);
    }

    public double getShooterVel(Pose2d robotPose, double robot_vx, double robot_vy, ArrayList<Pose3d> target){
        ArrayList<Pose3d> test;
        ArrayList<Double> cVs= new ArrayList<Double>();
        for(double cV = 20; cV<100; cV+=0.1){
            test = SimShot(cV,  robotPose,  robot_vx,  robot_vy);
            if(IsHit(test,target)){
                cVs.add(cV);
            }
        }

        if(cVs.size()>=1){
            double ctr = 0;
            for(int i = 0; i<cVs.size(); i++){
                ctr+= cVs.get(i);
            }
            return ctr/(cVs.size());
        }
        return -1;
    }
}
