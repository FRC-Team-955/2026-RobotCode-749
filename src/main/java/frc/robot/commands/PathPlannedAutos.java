package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.CANFuelSubsystem;

import static frc.robot.Constants.PoseConstants.INITIAL_POSE_LEFT_BUMP;

public class PathPlannedAutos {

    public static Command PathPlanLBumpShootX2(CANDriveSubsystem driveSubsystem, CANFuelSubsystem ballSubsystem){
        return  new SequentialCommandGroup(
                driveSubsystem.runOnce(()-> driveSubsystem.resetOdometry(INITIAL_POSE_LEFT_BUMP)),
                driveSubsystem.driveAtTargetPoseSup(()->ballSubsystem.poseToFaceHub()).withTimeout(1),
                ballSubsystem.shootAtTarget(0).withTimeout(4), //// TODO: tune the table!
                //ballSubsystem.launchCommand(() -> -6.63).withTimeout(4), //idk duplicate of above; use this is fable is bad
                ballSubsystem.runOnce(() -> ballSubsystem.stop()),
                driveSubsystem.driveAtTargetPoseSup(()->INITIAL_POSE_LEFT_BUMP).withTimeout(1),
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
}
