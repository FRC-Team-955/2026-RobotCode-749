package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.CANClimberSubsystem;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.CANFuelSubsystem;

import static frc.robot.Constants.PoseConstants.*;

public class ClimbAutos {
    public static Command centerToLClimbP2P(CANDriveSubsystem driveSubsystem, CANFuelSubsystem ballSubsystem, CANClimberSubsystem climberSubsystem) { // oh come on let me call it ds, fs, and cs please
        return new SequentialCommandGroup(
                driveSubsystem.runOnce(() -> driveSubsystem.resetOdometry(INITIAL_POSE_CENTER_HUB)),
                driveSubsystem.driveAtTargetPose(LEFT_CLIMB_POINT_ALIGN),
                climberSubsystem.goToTop().withTimeout(3),
                driveSubsystem.driveAtTargetPose(LEFT_CLIMB_POINT_FINAL).withTimeout(3), // potentially also drive forawrd a little?
                climberSubsystem.goToBottom().withTimeout(4) ///  TODO: IMPORTANT! tune this function (and goToTp())
        );
    }


    //TODO: uhh this doesnt look like it climbs
    public static Command centerShootClimb(CANDriveSubsystem driveSubsystem, CANFuelSubsystem ballSubsystem) {
        return new SequentialCommandGroup(
                driveSubsystem.runOnce(() -> driveSubsystem.resetOdometry(INITIAL_POSE_CENTER_HUB)),
                driveSubsystem.setPIDSetpoints(() -> -0.5, () -> -0.5),
                driveSubsystem.autoDrivePID(driveSubsystem.giveLeftEncoder(), driveSubsystem.giveRightEncoder()).withTimeout(2),
                ballSubsystem.spinUpCommand().until(() -> ballSubsystem.isAtSpeed(Constants.FuelConstants.SHOOTER_WEAK_SPEED)),
                ballSubsystem.launchCommand(() -> Constants.FuelConstants.SHOOTER_WEAK_LAUNCH_VOLTAGE).withTimeout(4),
                ballSubsystem.runOnce(() -> ballSubsystem.stop()),
                driveSubsystem.setPIDSetpoints(() -> -2.2, () -> -2.2),
                driveSubsystem.autoDrivePID(driveSubsystem.giveLeftEncoder(), driveSubsystem.giveRightEncoder()).withTimeout(2) // add later im lazy
        );
    }

}
