package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

// Just a class to contain annoying functions we dont wanna put everywhere related to game state!
// Also for stuff to be put on DS, to be accessd all over in the code
public class DSUtil {
    public static final Field2d FIELD = new Field2d();
    public static boolean isRedAlliance() {
        return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue).equals(DriverStation.Alliance.Red);
    }

    public static boolean isTeleop(){
        return DriverStation.isTeleopEnabled();
    }

    public static boolean isTestMode(){
        return DriverStation.isTestEnabled();
    }


}
