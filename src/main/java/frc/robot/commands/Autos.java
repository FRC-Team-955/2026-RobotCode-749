// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.subsystems.CANDriveSubsystem;

public final class Autos {
    // Example autonomous command which drives forward for 1 second.
    public static Command exampleAuto(CANDriveSubsystem driveSubsystem, CANFuelSubsystem ballSubsystem) {
        return new SequentialCommandGroup(
                // Drive backwards for .25 seconds. The driveArcadeAuto command factory
                // creates a command which does not end which allows us to control
                // the timing using the withTimeout decorator
                driveSubsystem.driveArcade(() -> 0.5, () -> 0).withTimeout(.25),  // make faster?
                // Stop driving. This line uses the regular driveArcade command factory so it
                // ends immediately after commanding the motors to stop
                driveSubsystem.driveArcade(() -> 0, () -> 0),
                // Spin up the launcher for 1 second and then launch balls for 9 seconds, for a
                // total of 10 seconds
                ballSubsystem.spinUpCommand().until(()->ballSubsystem.isAtSpeed()),
                ballSubsystem.launchCommand().withTimeout(2),
                // ballSubsystem.spinUpCommand().withTimeout(1), // longer spinup time for more consistency?
                // ballSubsystem.launchCommand().withTimeout(9),  // might not need
                // Stop running the launcher
                ballSubsystem.runOnce(() -> ballSubsystem.stop()));
    }

    public static Command PIDAuto(CANDriveSubsystem driveSubsystem, CANFuelSubsystem ballSubsystem) {
        return new SequentialCommandGroup(
                driveSubsystem.setPIDSetpoints(() -> 0.2, () -> -0.2), //????????????
                driveSubsystem.autoDrivePID()
        );
    }

    public static Command PIDRotateHalf(CANDriveSubsystem driveSubsystem, CANFuelSubsystem ballSubsystem) {
        return new SequentialCommandGroup(
                driveSubsystem.setPIDSetpoints(() -> 0.5, () -> 0.5),
                driveSubsystem.autoDrivePID()
        );
    }

    public static Command boringAuto(CANDriveSubsystem driveSubsystem, CANFuelSubsystem ballSubsystem) {
        return new SequentialCommandGroup(
                driveSubsystem.resetPIDSetpoints(),
               driveSubsystem.setPIDSetpoints(() -> -3, () -> -3),
          driveSubsystem.autoDrivePID(),
          ballSubsystem.spinUpCommand().until(() -> ballSubsystem.isAtSpeed()),
          ballSubsystem.launchCommand().withTimeout(2),
                ballSubsystem.runOnce(() -> ballSubsystem.stop())
        );
    }

    public static Command weakShoot(CANDriveSubsystem driveSubsystem, CANFuelSubsystem ballSubsystem) {
        return new SequentialCommandGroup(
             ballSubsystem.spinUpWeakCommand().until(() -> ballSubsystem.isAtWeakSpeed()),
                ballSubsystem.weakLaunchCommand().withTimeout(2),
                ballSubsystem.runOnce(() -> ballSubsystem.stop())
        );
    }


    public static Command ArinsPIDAuto(CANDriveSubsystem ds){
        return ds.driveAtTargetPose(new Pose2d(3,3,new Rotation2d()));
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