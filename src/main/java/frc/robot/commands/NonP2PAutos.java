package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.CANFuelSubsystem;

import static frc.robot.Constants.FuelConstants.SHOOTER_LAUNCH_VOLTAGE;
import static frc.robot.Constants.FuelConstants.SHOOTER_WEAK_SPEED;
import static frc.robot.Constants.PoseConstants.*;

public class NonP2PAutos {
    public static Command PIDAuto(CANDriveSubsystem driveSubsystem, CANFuelSubsystem ballSubsystem) {
        return new SequentialCommandGroup(
                driveSubsystem.setPIDSetpoints(() -> -1, () -> -1), // same sign i think
                driveSubsystem.autoDrivePID(driveSubsystem.giveLeftEncoder(), driveSubsystem.giveRightEncoder())
        );
    }

    public static Command PIDRotateHalf(CANDriveSubsystem driveSubsystem, CANFuelSubsystem ballSubsystem) {
        return new SequentialCommandGroup(
                driveSubsystem.setPIDSetpoints(() -> -1, () -> 0),
                driveSubsystem.autoDrivePID(driveSubsystem.giveLeftEncoder(), driveSubsystem.giveRightEncoder()).withTimeout(3),
                ballSubsystem.spinUpCommand().until(() -> ballSubsystem.isAtSpeed(Constants.FuelConstants.SHOOTER_WEAK_SPEED)),
                ballSubsystem.launchCommand(() -> Constants.FuelConstants.SHOOTER_WEAK_LAUNCH_VOLTAGE).withTimeout(4),
                ballSubsystem.runOnce(() -> ballSubsystem.stop())
        );
    }

    public static Command rBumpShoot(CANDriveSubsystem driveSubsystem, CANFuelSubsystem ballSubsystem) {
        return new SequentialCommandGroup(
                driveSubsystem.runOnce(() -> driveSubsystem.resetOdometry(INITIAL_POSE_RIGHT_BUMP)),
                driveSubsystem.setPIDSetpoints(() -> 0, () -> -0.67),
                driveSubsystem.autoDrivePID(driveSubsystem.giveLeftEncoder(), driveSubsystem.giveRightEncoder()).withTimeout(3),
                ballSubsystem.spinUpCommand().until(() -> ballSubsystem.isAtSpeed(48)),
                ballSubsystem.launchCommand(() -> -6.63).withTimeout(4),
                ballSubsystem.runOnce(() -> ballSubsystem.stop())
        );
    }

    public static Command lBumpShoot(CANDriveSubsystem driveSubsystem, CANFuelSubsystem ballSubsystem) {
        return new SequentialCommandGroup(
                driveSubsystem.runOnce(() -> driveSubsystem.resetOdometry(INITIAL_POSE_LEFT_BUMP)),
                driveSubsystem.setPIDSetpoints(() -> -0.67, () -> 0),
                driveSubsystem.autoDrivePID(driveSubsystem.giveLeftEncoder(), driveSubsystem.giveRightEncoder()).withTimeout(3),
                ballSubsystem.spinUpCommand().until(() -> ballSubsystem.isAtSpeed(48)),
                ballSubsystem.launchCommand(() -> -6.63).withTimeout(4),
                ballSubsystem.runOnce(() -> ballSubsystem.stop())
        );
    }

    public static Command expRBump(CANDriveSubsystem driveSubsystem, CANFuelSubsystem ballSubsystem) {
        return new SequentialCommandGroup(
                driveSubsystem.runOnce(() -> driveSubsystem.resetOdometry(INITIAL_POSE_RIGHT_BUMP)),
                driveSubsystem.setPIDSetpoints(() -> 0, () -> -0.67),
                driveSubsystem.autoDrivePID(driveSubsystem.giveLeftEncoder(), driveSubsystem.giveRightEncoder()).withTimeout(3),
                ballSubsystem.spinUpCommand().until(() -> ballSubsystem.isAtSpeed(48)),
                ballSubsystem.launchCommand(() -> -6.63).withTimeout(4),
                ballSubsystem.runOnce(() -> ballSubsystem.stop()), // everything after this is experimental
                driveSubsystem.setPIDSetpoints(() -> 0, () -> -0.2),
                driveSubsystem.autoDrivePID(driveSubsystem.giveLeftEncoder(), driveSubsystem.giveRightEncoder()).withTimeout(2),
                driveSubsystem.setPIDSetpoints(() -> 0.5, () -> 0.5),
                driveSubsystem.autoDrivePID(driveSubsystem.giveLeftEncoder(), driveSubsystem.giveRightEncoder()).withTimeout(2),
                driveSubsystem.setPIDSetpoints(() -> 0, () -> 0.9),
                driveSubsystem.autoDrivePID(driveSubsystem.giveLeftEncoder(), driveSubsystem.giveRightEncoder()).withTimeout(2)
        );
    }

    public static Command expLBump(CANDriveSubsystem driveSubsystem, CANFuelSubsystem ballSubsystem) {
        return new SequentialCommandGroup(
                driveSubsystem.runOnce(() -> driveSubsystem.resetOdometry(INITIAL_POSE_LEFT_BUMP)),
                driveSubsystem.setPIDSetpoints(() -> -0.67, () -> 0),
                driveSubsystem.autoDrivePID(driveSubsystem.giveLeftEncoder(), driveSubsystem.giveRightEncoder()).withTimeout(3),
                ballSubsystem.spinUpCommand().until(() -> ballSubsystem.isAtSpeed(48)),
                ballSubsystem.launchCommand(() -> -6.63).withTimeout(4),
                ballSubsystem.runOnce(() -> ballSubsystem.stop()), // everything after this is experimental
                driveSubsystem.setPIDSetpoints(() -> -0.2, () -> 0),
                driveSubsystem.autoDrivePID(driveSubsystem.giveLeftEncoder(), driveSubsystem.giveRightEncoder()).withTimeout(2),
                driveSubsystem.setPIDSetpoints(() -> 0.5, () -> 0.5),
                driveSubsystem.autoDrivePID(driveSubsystem.giveLeftEncoder(), driveSubsystem.giveRightEncoder()).withTimeout(2),
                driveSubsystem.setPIDSetpoints(() -> 0.9, () -> 0),
                driveSubsystem.autoDrivePID(driveSubsystem.giveLeftEncoder(), driveSubsystem.giveRightEncoder()).withTimeout(2)
        );
    }


    public static Command centerShoot(CANDriveSubsystem driveSubsystem, CANFuelSubsystem ballSubsystem) {
        return new SequentialCommandGroup(
                driveSubsystem.runOnce(() -> driveSubsystem.resetOdometry(INITIAL_POSE_CENTER_HUB)),
                driveSubsystem.setPIDSetpoints(() -> -0.5, () -> -0.5),
                driveSubsystem.autoDrivePID(driveSubsystem.giveLeftEncoder(), driveSubsystem.giveRightEncoder()).withTimeout(2),
                ballSubsystem.spinUpCommand().until(() -> ballSubsystem.isAtSpeed(Constants.FuelConstants.SHOOTER_WEAK_SPEED)),
                ballSubsystem.launchCommand(() -> Constants.FuelConstants.SHOOTER_WEAK_LAUNCH_VOLTAGE).withTimeout(4),
                ballSubsystem.runOnce(() -> ballSubsystem.stop())
        );
    }



    public static Command expCenterShootL(CANDriveSubsystem driveSubsystem, CANFuelSubsystem ballSubsystem) {
        return new SequentialCommandGroup(
                driveSubsystem.runOnce(() -> driveSubsystem.resetOdometry(INITIAL_POSE_CENTER_HUB)),
                driveSubsystem.setPIDSetpoints(() -> -0.5, () -> -0.5),
                driveSubsystem.autoDrivePID(driveSubsystem.giveLeftEncoder(), driveSubsystem.giveRightEncoder()).withTimeout(2),
                ballSubsystem.spinUpCommand().until(() -> ballSubsystem.isAtSpeed(Constants.FuelConstants.SHOOTER_WEAK_SPEED)),
                ballSubsystem.launchCommand(() -> Constants.FuelConstants.SHOOTER_WEAK_LAUNCH_VOLTAGE).withTimeout(4),
                ballSubsystem.runOnce(() -> ballSubsystem.stop()),
                driveSubsystem.setPIDSetpoints(() -> 0.4, () -> 0),
                driveSubsystem.autoDrivePID(driveSubsystem.giveLeftEncoder(), driveSubsystem.giveRightEncoder()).withTimeout(2),
                driveSubsystem.setPIDSetpoints(() -> 0.5, () -> 0.5),
                driveSubsystem.autoDrivePID(driveSubsystem.giveLeftEncoder(), driveSubsystem.giveRightEncoder()).withTimeout(2),
                driveSubsystem.setPIDSetpoints(() -> 0, () -> 0.4),
                driveSubsystem.autoDrivePID(driveSubsystem.giveLeftEncoder(), driveSubsystem.giveRightEncoder()).withTimeout(2)
        );
    }

    public static Command expCenterShootR(CANDriveSubsystem driveSubsystem, CANFuelSubsystem ballSubsystem) {
        return new SequentialCommandGroup(
                driveSubsystem.runOnce(() -> driveSubsystem.resetOdometry(INITIAL_POSE_CENTER_HUB)),
                driveSubsystem.setPIDSetpoints(() -> -0.5, () -> -0.5),
                driveSubsystem.autoDrivePID(driveSubsystem.giveLeftEncoder(), driveSubsystem.giveRightEncoder()).withTimeout(2),
                ballSubsystem.spinUpCommand().until(() -> ballSubsystem.isAtSpeed(Constants.FuelConstants.SHOOTER_WEAK_SPEED)),
                ballSubsystem.launchCommand(() -> Constants.FuelConstants.SHOOTER_WEAK_LAUNCH_VOLTAGE).withTimeout(4),
                ballSubsystem.runOnce(() -> ballSubsystem.stop()),
                driveSubsystem.setPIDSetpoints(() -> -0.53, () -> 0.53),
                driveSubsystem.autoDrivePID(driveSubsystem.giveLeftEncoder(), driveSubsystem.giveRightEncoder()).withTimeout(2),
                driveSubsystem.setPIDSetpoints(() -> 1.5, () -> 1.5),
                driveSubsystem.autoDrivePID(driveSubsystem.giveLeftEncoder(), driveSubsystem.giveRightEncoder()).withTimeout(2),
                driveSubsystem.setPIDSetpoints(() -> -0.55, () -> 0.55),
                driveSubsystem.autoDrivePID(driveSubsystem.giveLeftEncoder(), driveSubsystem.giveRightEncoder()).withTimeout(2),
                driveSubsystem.setPIDSetpoints(() -> -3, () -> -3),
                driveSubsystem.autoSlowDrivePID(driveSubsystem.giveLeftEncoder(), driveSubsystem.giveRightEncoder()).withTimeout(2),
                driveSubsystem.setPIDSetpoints(() -> -0.75, () -> -0.75),
                driveSubsystem.autoDrivePID(driveSubsystem.giveLeftEncoder(), driveSubsystem.giveRightEncoder()).withTimeout(2),
                driveSubsystem.setPIDSetpoints(() -> 0, () -> -1.2),
                driveSubsystem.autoDrivePID(driveSubsystem.giveLeftEncoder(), driveSubsystem.giveRightEncoder()).withTimeout(2),
                driveSubsystem.setPIDSetpoints(() -> -0.4, () -> -1.4),
                driveSubsystem.autoDrivePID(driveSubsystem.giveLeftEncoder(), driveSubsystem.giveRightEncoder()).withTimeout(2),
                driveSubsystem.setPIDSetpoints(() -> -0.4, () -> -0.4),
                driveSubsystem.autoDrivePID(driveSubsystem.giveLeftEncoder(), driveSubsystem.giveRightEncoder()).withTimeout(2),
                driveSubsystem.setPIDSetpoints(() -> -0.8, () -> -2.8),
                driveSubsystem.autoDrivePID(driveSubsystem.giveLeftEncoder(), driveSubsystem.giveRightEncoder()).withTimeout(2),
                driveSubsystem.setPIDSetpoints(() -> -1.75, () -> -1.75),
                driveSubsystem.autoDrivePID(driveSubsystem.giveLeftEncoder(), driveSubsystem.giveRightEncoder()).withTimeout(2)
//                driveSubsystem.setPIDSetpoints(() -> -1.2, () -> 0),
//                driveSubsystem.autoDrivePID(driveSubsystem.giveLeftEncoder(), driveSubsystem.giveRightEncoder()).withTimeout(2),
//                driveSubsystem.setPIDSetpoints(() -> -3, () -> -3),
//                driveSubsystem.autoDrivePID(driveSubsystem.giveLeftEncoder(), driveSubsystem.giveRightEncoder()).withTimeout(2),
//                driveSubsystem.setPIDSetpoints(() -> 0.55, () -> -0.55),
//                driveSubsystem.autoDrivePID(driveSubsystem.giveLeftEncoder(), driveSubsystem.giveRightEncoder()).withTimeout(2),
//                driveSubsystem.setPIDSetpoints(() -> 1.5, () -> -1.5),
//                driveSubsystem.autoDrivePID(driveSubsystem.giveLeftEncoder(), driveSubsystem.giveRightEncoder()).withTimeout(2),
//                driveSubsystem.setPIDSetpoints(() -> 0.53, () -> -0.53),
//                driveSubsystem.autoDrivePID(driveSubsystem.giveLeftEncoder(), driveSubsystem.giveRightEncoder()).withTimeout(2),
//                ballSubsystem.spinUpCommand().until(() -> ballSubsystem.isAtSpeed(Constants.FuelConstants.SHOOTER_WEAK_SPEED)),
//                ballSubsystem.launchCommand(() -> Constants.FuelConstants.SHOOTER_WEAK_LAUNCH_VOLTAGE).withTimeout(4),
//                ballSubsystem.runOnce(() -> ballSubsystem.stop())



        );
    }

    public static Command boringAuto(CANDriveSubsystem driveSubsystem, CANFuelSubsystem ballSubsystem) {
        return new SequentialCommandGroup(
                driveSubsystem.setPIDSetpoints(() -> -3, () -> -3),
                driveSubsystem.autoDrivePID(driveSubsystem.giveLeftEncoder(), driveSubsystem.giveRightEncoder()),
                ballSubsystem.spinUpCommand().until(() -> ballSubsystem.isAtSpeed(Constants.FuelConstants.SHOOTER_STRONG_SPEED)),
                ballSubsystem.launchCommand(() -> SHOOTER_LAUNCH_VOLTAGE).withTimeout(2),
                ballSubsystem.runOnce(() -> ballSubsystem.stop())
        );
    }

    public static Command weakShoot(CANDriveSubsystem driveSubsystem, CANFuelSubsystem ballSubsystem) {
        return new SequentialCommandGroup(
                driveSubsystem.runOnce(() -> driveSubsystem.resetOdometry(INITIAL_POSE_CENTER_HUB)),
                driveSubsystem.setPIDSetpoints(() -> -0.5, () -> -0.5),
                driveSubsystem.autoDrivePID(driveSubsystem.giveLeftEncoder(), driveSubsystem.giveRightEncoder()).withTimeout(2),
                ballSubsystem.spinUpCommand().until(() -> ballSubsystem.isAtSpeed(SHOOTER_WEAK_SPEED)),
                ballSubsystem.launchCommand(() -> Constants.FuelConstants.SHOOTER_WEAK_LAUNCH_VOLTAGE).withTimeout(4),
                ballSubsystem.runOnce(() -> ballSubsystem.stop())
        );
    }
}
