package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CANClimberSubsystem;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.CANFuelSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import static frc.robot.Constants.PoseConstants.*;
import static frc.robot.Constants.PoseConstants.LEFT_CLIMB_POINT_ALIGN;
import static frc.robot.Constants.PoseConstants.LEFT_CLIMB_POINT_FINAL;

public class PathPlannedAutos {

    public static Command PathPlanLBumpShootX2(CANDriveSubsystem driveSubsystem, CANFuelSubsystem ballSubsystem){
        return  new SequentialCommandGroup(
                driveSubsystem.runOnce(()-> driveSubsystem.resetOdometry(INITIAL_POSE_LEFT_BUMP)),
                driveSubsystem.driveAtTargetPoseSup(()->ballSubsystem.poseToFaceHub()).withTimeout(1),
                ballSubsystem.shootAtTarget(0).withTimeout(4), //// TODO: tune the table!
                //ballSubsystem.launchCommand(() -> -6.63).withTimeout(4), //idk duplicate of above; use this is fable is bad
                ballSubsystem.runOnce(() -> ballSubsystem.stop()),
                driveSubsystem.driveAtTargetPoseSup(()->new Pose2d(INITIAL_POSE_LEFT_BUMP.getTranslation(),Rotation2d.kZero)).withTimeout(1),
                new ParallelRaceGroup(
                            AutoBuilder.buildAuto("Left Bump Shoot"),
                            ballSubsystem.run(() -> ballSubsystem.intake())
                        ),
                driveSubsystem.driveAtTargetPoseSup(()->ballSubsystem.poseToFaceHub()).withTimeout(1),
                ballSubsystem.shootAtTarget(0).withTimeout(4), //// TODO: tune the table!
                //ballSubsystem.launchCommand(() -> -6.63).withTimeout(4), //idk duplicate of above; use this is fable is bad
                ballSubsystem.runOnce(() -> ballSubsystem.stop())
        );
    }

    public static Command PathPlanRBumpShootX2(CANDriveSubsystem driveSubsystem, CANFuelSubsystem ballSubsystem){
        return  new SequentialCommandGroup(
                driveSubsystem.runOnce(()-> driveSubsystem.resetOdometry(INITIAL_POSE_RIGHT_BUMP)),
                driveSubsystem.driveAtTargetPoseSup(()->ballSubsystem.poseToFaceHub()).withTimeout(1),
                ballSubsystem.shootAtTarget(0).withTimeout(4), //// TODO: tune the table!
                //ballSubsystem.launchCommand(() -> -6.63).withTimeout(4), //idk duplicate of above; use this is fable is bad
                ballSubsystem.runOnce(() -> ballSubsystem.stop()),
                driveSubsystem.driveAtTargetPoseSup(()->new Pose2d(INITIAL_POSE_RIGHT_BUMP.getTranslation(),Rotation2d.kZero)).withTimeout(1),
                new ParallelRaceGroup(
                        AutoBuilder.buildAuto("Right Bump Shoot"),
                        ballSubsystem.run(() -> ballSubsystem.intake())
                ),
                driveSubsystem.driveAtTargetPoseSup(()->ballSubsystem.poseToFaceHub()).withTimeout(1),
                ballSubsystem.shootAtTarget(0).withTimeout(4), //// TODO: tune the table!
                //ballSubsystem.launchCommand(() -> -6.63).withTimeout(4), //idk duplicate of above; use this is fable is bad
                ballSubsystem.runOnce(() -> ballSubsystem.stop())
        );
    }

    public static Command PathPlanLClimb(CANDriveSubsystem driveSubsystem, CANClimberSubsystem climberSubsystem){
        return new SequentialCommandGroup(
        driveSubsystem.runOnce(() -> driveSubsystem.resetOdometry(INITIAL_POSE_CENTER_HUB)),
                AutoBuilder.buildAuto("Left Climb"),
                climberSubsystem.goToTop().withTimeout(3),
                driveSubsystem.driveAtTargetPose(LEFT_CLIMB_POINT_FINAL).withTimeout(3), // potentially also drive forawrd a little?
                climberSubsystem.goToBottom().withTimeout(4) ///  TODO: IMPORTANT! tune this function (and goToTp())
        );
    }
}
