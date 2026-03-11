// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.subsystems.CANDriveSubsystem;

import static frc.robot.RobotState.INITIAL_POSE;

public final class Autos {
    // Example autonomous command which drives forward for 1 second.
    public static Command exampleAuto(CANDriveSubsystem driveSubsystem, CANFuelSubsystem ballSubsystem) {
        return new SequentialCommandGroup(
                // Drive backwards for .25 seconds. The driveArcadeAuto command factory
                // creates a command which does not end which allows us to control
                // the timing using the withTimeout decorator
                driveSubsystem.driveArcade(() -> 0.5, () -> 0, () -> 0).withTimeout(.25),  // make faster?
                // Stop driving. This line uses the regular driveArcade command factory so it
                // ends immediately after commanding the motors to stop
                driveSubsystem.driveArcade(() -> 0, () -> 0, () -> 0),
                // Spin up the launcher for 1 second and then launch balls for 9 seconds, for a
                // total of 10 seconds
                ballSubsystem.spinUpCommand().until(()->ballSubsystem.isAtSpeed(Constants.FuelConstants.SHOOTER_STRONG_SPEED)),
                ballSubsystem.launchCommand(() -> Constants.FuelConstants.SHOOTER_LAUNCH_VOLTAGE).withTimeout(2),
                // ballSubsystem.spinUpCommand().withTimeout(1), // longer spinup time for more consistency?
                // ballSubsystem.launchCommand().withTimeout(9),  // might not need
                // Stop running the launcher
                ballSubsystem.runOnce(() -> ballSubsystem.stop()));
    }

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
                driveSubsystem.setPIDSetpoints(() -> 0, () -> -0.67),
                driveSubsystem.autoDrivePID(driveSubsystem.giveLeftEncoder(), driveSubsystem.giveRightEncoder()).withTimeout(3),
                ballSubsystem.spinUpCommand().until(() -> ballSubsystem.isAtSpeed(48)),
                ballSubsystem.launchCommand(() -> -6.63).withTimeout(4),
                ballSubsystem.runOnce(() -> ballSubsystem.stop())
        );
    }

    public static Command lBumpShoot(CANDriveSubsystem driveSubsystem, CANFuelSubsystem ballSubsystem) {
        return new SequentialCommandGroup(
                driveSubsystem.setPIDSetpoints(() -> -0.67, () -> 0),
                driveSubsystem.autoDrivePID(driveSubsystem.giveLeftEncoder(), driveSubsystem.giveRightEncoder()).withTimeout(3),
                ballSubsystem.spinUpCommand().until(() -> ballSubsystem.isAtSpeed(48)),
                ballSubsystem.launchCommand(() -> -6.63).withTimeout(4),
                ballSubsystem.runOnce(() -> ballSubsystem.stop())
        );
    }

    public static Command expRBump(CANDriveSubsystem driveSubsystem, CANFuelSubsystem ballSubsystem) {
        return new SequentialCommandGroup(
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

    public static Command lBumpShootP2P(CANDriveSubsystem driveSubsystem, CANFuelSubsystem ballSubsystem) {
        return new SequentialCommandGroup(
                driveSubsystem.runOnce(()-> driveSubsystem.resetOdometry(new Pose2d(3.66,6,Rotation2d.k180deg))),
                driveSubsystem.driveAtTargetPose(new Pose2d(3.66,6, new Rotation2d(2.02))),
                ballSubsystem.spinUpCommand().until(() -> ballSubsystem.isAtSpeed(48)), // EVENTUALLY replace with ballSubsystem.shootAtTarget()!!!!
                ballSubsystem.launchCommand(() -> -6.63).withTimeout(4),
                ballSubsystem.runOnce(() -> ballSubsystem.stop())
        );
    }

    public static Command centerShoot(CANDriveSubsystem driveSubsystem, CANFuelSubsystem ballSubsystem) {
        return new SequentialCommandGroup(
                driveSubsystem.setPIDSetpoints(() -> -0.5, () -> -0.5),
                driveSubsystem.autoDrivePID(driveSubsystem.giveLeftEncoder(), driveSubsystem.giveRightEncoder()).withTimeout(2),
                ballSubsystem.spinUpCommand().until(() -> ballSubsystem.isAtSpeed(Constants.FuelConstants.SHOOTER_WEAK_SPEED)),
                ballSubsystem.launchCommand(() -> Constants.FuelConstants.SHOOTER_WEAK_LAUNCH_VOLTAGE).withTimeout(4),
                ballSubsystem.runOnce(() -> ballSubsystem.stop())
        );
    }

    public static Command expCenterShootL(CANDriveSubsystem driveSubsystem, CANFuelSubsystem ballSubsystem) {
        return new SequentialCommandGroup(
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
                driveSubsystem.setPIDSetpoints(() -> -0.5, () -> -0.5),
                driveSubsystem.autoDrivePID(driveSubsystem.giveLeftEncoder(), driveSubsystem.giveRightEncoder()).withTimeout(2),
                ballSubsystem.spinUpCommand().until(() -> ballSubsystem.isAtSpeed(Constants.FuelConstants.SHOOTER_WEAK_SPEED)),
                ballSubsystem.launchCommand(() -> Constants.FuelConstants.SHOOTER_WEAK_LAUNCH_VOLTAGE).withTimeout(4),
                ballSubsystem.runOnce(() -> ballSubsystem.stop()),
                driveSubsystem.setPIDSetpoints(() -> 0, () -> 0.4),
                driveSubsystem.autoDrivePID(driveSubsystem.giveLeftEncoder(), driveSubsystem.giveRightEncoder()).withTimeout(2),
                driveSubsystem.setPIDSetpoints(() -> 0.5, () -> 0.5),
                driveSubsystem.autoDrivePID(driveSubsystem.giveLeftEncoder(), driveSubsystem.giveRightEncoder()).withTimeout(2),
                driveSubsystem.setPIDSetpoints(() -> 0.4, () -> 0),
                driveSubsystem.autoDrivePID(driveSubsystem.giveLeftEncoder(), driveSubsystem.giveRightEncoder()).withTimeout(2)
        );
    }

    public static Command boringAuto(CANDriveSubsystem driveSubsystem, CANFuelSubsystem ballSubsystem) {
        return new SequentialCommandGroup(
               driveSubsystem.setPIDSetpoints(() -> -3, () -> -3),
          driveSubsystem.autoDrivePID(driveSubsystem.giveLeftEncoder(), driveSubsystem.giveRightEncoder()),
          ballSubsystem.spinUpCommand().until(() -> ballSubsystem.isAtSpeed(Constants.FuelConstants.SHOOTER_STRONG_SPEED)),
          ballSubsystem.launchCommand(() -> Constants.FuelConstants.SHOOTER_LAUNCH_VOLTAGE).withTimeout(2),
                ballSubsystem.runOnce(() -> ballSubsystem.stop())
        );
    }

    public static Command weakShoot(CANDriveSubsystem driveSubsystem, CANFuelSubsystem ballSubsystem) {
        return new SequentialCommandGroup(
                driveSubsystem.setPIDSetpoints(() -> -0.5, () -> -0.5),
                driveSubsystem.autoDrivePID(driveSubsystem.giveLeftEncoder(), driveSubsystem.giveRightEncoder()).withTimeout(2),
             ballSubsystem.spinUpCommand().until(() -> ballSubsystem.isAtSpeed(Constants.FuelConstants.SHOOTER_WEAK_SPEED)),
                ballSubsystem.launchCommand(() -> Constants.FuelConstants.SHOOTER_WEAK_LAUNCH_VOLTAGE).withTimeout(4),
                ballSubsystem.runOnce(() -> ballSubsystem.stop())
        );
    }


    public static Command P2PAutoTest(CANDriveSubsystem ds, CANFuelSubsystem fs){
        return new SequentialCommandGroup(
                ds.runOnce(()-> ds.resetOdometry(INITIAL_POSE)), // correctly set Auto's start pos, in sim and irl!
                ds.driveAtTargetPose(new Pose2d(2.8,4, new Rotation2d(Math.PI) )),
                fs.spinUpCommand().until(() -> fs.isAtSpeed(Constants.FuelConstants.SHOOTER_STRONG_SPEED)).withTimeout(2),
                fs.shootAtTarget(0).withTimeout(6),
                fs.runOnce(()->fs.stop()),
                ds.driveAtTargetPose(new Pose2d(3.633,5.41, Rotation2d.kZero)),
                ds.driveAtTargetPose(new Pose2d(8.00,5.41, new Rotation2d(3*Math.PI/2))).alongWith(fs.run(()->fs.intake())),
                fs.runOnce(()->fs.stop())
        );
    }


    public static Command ChaosTheoryAuto(CANDriveSubsystem ds, CANFuelSubsystem fs){
        return new SequentialCommandGroup(
                ds.runOnce(()-> ds.resetOdometry(INITIAL_POSE)), // correctly set Auto's start pos, in sim and irl!
                ds.driveAtTargetPose(new Pose2d(3.6,4, new Rotation2d(fs.toFaceHub()) )),
                ds.driveAtTargetPose(new Pose2d(0.3,0.7, new Rotation2d() )),
                fs.run(()->fs.intake()).withTimeout(6),
                ds.driveAtTargetPose(new Pose2d(0.8,1, new Rotation2d(0.6+Math.PI) )),
                fs.shootAtTarget(Constants.FuelConstants.CORNER_HIT_VELOCITY).withTimeout(13),
                fs.runOnce(()->fs.stop())

        );
    }

  /*
  public static Command initalAuto(CANDriveSubsystem driveSubsystem, CANFuelSubsystem ballSubsystem){
  // turn from start pos to face hub
  // unload init balls

  // let's map this out in a drawing before doing any code*
  //      -use a bird's eye field model drawing with a correctly scaled robot
  //       (I can help you do that in Onshape) -Mathis
    return new SequentialCommandGroup();
  }
  */


}