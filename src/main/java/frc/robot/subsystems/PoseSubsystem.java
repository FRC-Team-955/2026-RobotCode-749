package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;

public class PoseSubsystem {

    private Pose2d BestGuess;


    public void resetOdometry(Pose2d a){
        BestGuess = a;
        return;
    }

    public Pose2d getPose(){
        return BestGuess;
    }
}
