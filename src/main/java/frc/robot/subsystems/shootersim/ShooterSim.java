package frc.robot.subsystems.shootersim;

public class ShooterSim {
    private double SHOOTER_X = 0.5; //offset from middle of robot
    private double SHOOTER_Y = 0;
    private double SHOOTER_Z = 0.4;
    private double SHOOTER_LAUNCH_THETA = 2*Math.PI/5; //Radians from horiz!
    InterpolationTable Table = new InterpolationTable(); // this thing is meant to track voltage vs landing point!
    public ShooterSim(){
        Table.add(0,0);
        Table.add(12,15); //meters
    }


    private void SimShot(){

    }
}
