package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;

// Just a class to contain annoying functions we dont wanna put everywhere related to game state!
public class DSUtil {

    public static boolean isRedAlliance() {
        return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue).equals(DriverStation.Alliance.Red);
    }

    public static boolean isTeleop(){
        return DriverStation.isTeleopEnabled();
    }


}
