// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.subsystems.CANDriveSubsystem;

import static frc.robot.Constants.PoseConstants.*;

public final class Autos {



    public static Command lBumpShoot(CANDriveSubsystem driveSubsystem, CANFuelSubsystem ballSubsystem) {
        return new SequentialCommandGroup(
                driveSubsystem.runOnce(()-> driveSubsystem.resetOdometry(INITIAL_POSE_LEFT_BUMP)),
                driveSubsystem.driveAtTargetPose(new Pose2d(INITIAL_POSE_LEFT_BUMP.getX()-0.8, INITIAL_POSE_LEFT_BUMP.getY()-0.5, new Rotation2d(ballSubsystem.toFaceHub(INITIAL_POSE_LEFT_BUMP.getX(), INITIAL_POSE_LEFT_BUMP.getY())))),
                //Safeguard spin up with a timeout! This makes simming possible!
                ballSubsystem.shootAtTarget(0).withTimeout(4), //// TODO: tune the table!
                ballSubsystem.launchCommand(() -> -6.63).withTimeout(4),
                ballSubsystem.runOnce(() -> ballSubsystem.stop()),

                driveSubsystem.driveAtTargetPose(new Pose2d(INITIAL_POSE_LEFT_BUMP.getX(), INITIAL_POSE_LEFT_BUMP.getY(), new Rotation2d())),
                new ParallelRaceGroup(
                        ballSubsystem.run(() -> ballSubsystem.intake()),
                        new SequentialCommandGroup(
                                driveSubsystem.driveAtTargetPose(new Pose2d(8.27, INITIAL_POSE_LEFT_BUMP.getY(), new Rotation2d())),
                                driveSubsystem.driveAtTargetPose(new Pose2d(8.27, 4.03, new Rotation2d(3.0 / 2 * Math.PI))),
                                driveSubsystem.driveAtTargetPose(new Pose2d(6.7, 4.03, new Rotation2d(Math.PI))),
                                driveSubsystem.driveAtTargetPose(new Pose2d(5.5, (INITIAL_POSE_LEFT_BUMP.getY()-0.5), new Rotation2d(Math.atan2((INITIAL_POSE_LEFT_BUMP.getY()-0.2-4.03), (4.4-6.7))))),
                                driveSubsystem.driveAtTargetPose(new Pose2d(3.8,  (INITIAL_POSE_LEFT_BUMP.getY()-0.5), new Rotation2d(Math.PI))),
                                driveSubsystem.driveAtTargetPose(new Pose2d(3.5, (INITIAL_POSE_LEFT_BUMP.getY()-0.8), new Rotation2d(ballSubsystem.toFaceHub())))
                        )
                ),
                ballSubsystem.runOnce(()->ballSubsystem.stop()),
                driveSubsystem.driveAtTargetPoseSup(()->ballSubsystem.poseToFaceHub()),
                ballSubsystem.shootAtTarget(0).withTimeout(4), // TODO: tune the table!
                ballSubsystem.launchCommand(() -> -6.63).withTimeout(4),
                ballSubsystem.runOnce(() -> ballSubsystem.stop())
        );
    }




    public static Command P2PTest(CANDriveSubsystem ds, CANFuelSubsystem fs){
        return new SequentialCommandGroup(
                ds.runOnce(()-> ds.resetOdometry(INITIAL_POSE_CENTER_HUB)), // correctly set Auto's start pos, in sim and irl!
                ds.driveAtTargetPose(new Pose2d(2.8,4, new Rotation2d(Math.PI) )),
                fs.spinUpCommand().until(() -> fs.isAtSpeed(Constants.FuelConstants.SHOOTER_STRONG_SPEED)).withTimeout(2),
                fs.shootAtTarget(0).withTimeout(6),
                fs.runOnce(()->fs.stop()),
                ds.driveAtTargetPose(new Pose2d(3.633,5.41, Rotation2d.kZero)),
                ds.driveAtTargetPose(new Pose2d(8.00,5.41, new Rotation2d(3*Math.PI/2))).alongWith(fs.run(()->fs.intake())),
                fs.runOnce(()->fs.stop())
        );
    }


    public static Command ChaosTheoryAuto(CANDriveSubsystem ds, CANFuelSubsystem fs) {
        return new SequentialCommandGroup(
                ds.runOnce(()-> ds.resetOdometry(INITIAL_POSE_CENTER_HUB)), // correctly set Auto's start pos, in sim and irl!
                ds.driveAtTargetPose(new Pose2d(3.6,4, new Rotation2d(fs.toFaceHub()) )),
                ds.driveAtTargetPose(new Pose2d(0.3,0.7, new Rotation2d() )),
                fs.run(()->fs.intake()).withTimeout(6),
                ds.driveAtTargetPose(new Pose2d(0.8,1, new Rotation2d(0.6+Math.PI) )),
                fs.shootAtTarget(Constants.FuelConstants.CORNER_HIT_VELOCITY).withTimeout(13),
                fs.runOnce(()->fs.stop())
        );
    }





}