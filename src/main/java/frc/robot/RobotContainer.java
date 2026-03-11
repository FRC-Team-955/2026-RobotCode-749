// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


import static frc.robot.Constants.OperatorConstants.*;
import static frc.robot.RobotState.INITIAL_POSE;

import frc.robot.commands.Autos;
import frc.robot.subsystems.CANClimberSubsystem;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.subsystems.PoseSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems
    private CANDriveSubsystem driveSubsystem;
    private CANFuelSubsystem ballSubsystem;
    private CANClimberSubsystem climberSubsystem;
    private PoseSubsystem poseSubsystem;




    // The driver's controller
    private final CommandXboxController driverController = new CommandXboxController(
            DRIVER_CONTROLLER_PORT);

    // The operator's controller
    private final CommandXboxController operatorController = new CommandXboxController(
            OPERATOR_CONTROLLER_PORT);

    // The autonomous chooser
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();


    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer(CANFuelSubsystem fuelSubsystem, CANDriveSubsystem driveSubsystem, PoseSubsystem poseSubsystem, CANClimberSubsystem climbersubsystem) {

    this.ballSubsystem = fuelSubsystem;
    this.driveSubsystem = driveSubsystem;
    this.poseSubsystem = poseSubsystem;
    this.climberSubsystem = climbersubsystem;





    poseSubsystem.resetOdometry(INITIAL_POSE);  /// TODO: TUNE THIS


        configureBindings(); //controller bindings


        // Set the options to show up in the Dashboard for selecting auto modes. If you
        // add additional auto modes you can add additional lines here with
        // autoChooser.addOption
        autoChooser.setDefaultOption("Center Weak Shoot", Autos.centerShoot(driveSubsystem, ballSubsystem));
        autoChooser.addOption("Right Bump Shoot", Autos.rBumpShoot(driveSubsystem, ballSubsystem));
        autoChooser.addOption("Left Bump Shoot", Autos.lBumpShoot(driveSubsystem, ballSubsystem));
        autoChooser.addOption("Center Weak Shoot", Autos.centerShoot(driveSubsystem, ballSubsystem));
        autoChooser.addOption("[EXP] Left Bump Shoot", Autos.expLBump(driveSubsystem, ballSubsystem));
        autoChooser.addOption("[EXP] Right Bump Shoot", Autos.expRBump(driveSubsystem, ballSubsystem));
        autoChooser.addOption("[EXP] Center Weak Shoot Left", Autos.expCenterShootL(driveSubsystem, ballSubsystem));
        autoChooser.addOption("[EXP] Center Weak Shoot Right", Autos.expCenterShootR(driveSubsystem, ballSubsystem));
        autoChooser.addOption("[TEST] P2P Left Bump Shoot", Autos.lBumpShootP2P(driveSubsystem, ballSubsystem));
        autoChooser.addOption("[TEST] PID 1m Auto", Autos.PIDAuto(driveSubsystem, ballSubsystem));
        autoChooser.addOption("[TEST] PID rotate bashy", Autos.PIDRotateHalf(driveSubsystem, ballSubsystem));
        autoChooser.addOption("[TEST] go back then shoot", Autos.boringAuto(driveSubsystem, ballSubsystem));
        autoChooser.addOption("[TEST] weak shoot", Autos.weakShoot(driveSubsystem, ballSubsystem));
        autoChooser.addOption("[TEST] P2P AUTO", Autos.P2PAutoTest(driveSubsystem, ballSubsystem));
        autoChooser.addOption("[TEST] CHAOS THEORY AUTO", Autos.ChaosTheoryAuto(driveSubsystem, ballSubsystem));
        SmartDashboard.putData(autoChooser);
    }


    private void configureBindings() {

        driverController.x()
                        .onTrue(climberSubsystem.runOnce(() -> climberSubsystem.changeMode()));

        driverController.b() // requirement is must be in tuning mode
                        .onTrue(climberSubsystem.runOnce(() -> climberSubsystem.setTopValue()));
        driverController.a() // requirement is must be in tuning mode
                .onTrue(climberSubsystem.runOnce(() -> climberSubsystem.setBottomValue()));

        driverController.rightBumper()
                        .whileTrue(climberSubsystem.runEnd(() -> climberSubsystem.goUp(), () -> climberSubsystem.stop()));

        driverController.leftBumper()
                        .whileTrue(climberSubsystem.runEnd(() -> climberSubsystem.goDown(), () -> climberSubsystem.stop()));

        // While the left bumper on operator controller is held, intake Fuel
        operatorController.leftBumper()
                .whileTrue(ballSubsystem.runEnd(() -> ballSubsystem.intake(), () -> ballSubsystem.stop()));

        driverController.y().whileTrue(ballSubsystem.shootAtTarget(0).finallyDo(()->ballSubsystem.stop()));
        // sorry not sorry arin :(

        //driverController.y().onTrue(driveSubsystem.shake());

        // While the right bumper on the operator controller is held, spin up for 1
        // second, then launch fuel. When the button is released, stop.
        operatorController.rightBumper()
                .whileTrue(ballSubsystem.spinUpCommand().until(()->ballSubsystem.isAtSpeed(Constants.FuelConstants.SHOOTER_STRONG_SPEED))
                        .andThen(ballSubsystem.launchCommand(() -> Constants.FuelConstants.SHOOTER_LAUNCH_VOLTAGE))
                        .finallyDo(() -> ballSubsystem.stop()));

        operatorController.y()
                .whileTrue(ballSubsystem.spinUpCommand().until(()->ballSubsystem.isAtSpeed(Constants.FuelConstants.SHOOTER_WEAK_SPEED))
                        .andThen(ballSubsystem.launchCommand(() -> Constants.FuelConstants.SHOOTER_WEAK_LAUNCH_VOLTAGE))
                        .finallyDo(() -> ballSubsystem.stop()));

        // While the A button is held on the operator controller, eject fuel back out
        // the intake
        operatorController.a()
                .whileTrue(ballSubsystem.runEnd(() -> ballSubsystem.eject(), () -> ballSubsystem.stop()));

        operatorController.x().onTrue(driveSubsystem.runOnce(()->driveSubsystem.resetOdometry(INITIAL_POSE))); // op x on click is restart poseSubsystem

        operatorController.povUp()
                .onTrue(driveSubsystem.runOnce(() -> driveSubsystem.increaseSens()));

        operatorController.povDown()
                .onTrue(driveSubsystem.runOnce(() -> driveSubsystem.decreaseSens()));

        operatorController.povLeft()
                .onTrue(driveSubsystem.runOnce(() -> driveSubsystem.sens100()));

        operatorController.povRight()
                .onTrue(driveSubsystem.runOnce(() -> driveSubsystem.sens50()));


        // Set the default command for the drive subsystem to the command provided by
        // factory with the values provided by the joystick axes on the driver
        // controller. The Y axis of the controller is inverted so that pushing the
        // stick away from you (a negative value) drives the robot forwards (a positive
        // value). The X-axis is also inverted so a positive value (stick to the right)
        // results in clockwise rotation (front of the robot turning right). Both axes
        // are also scaled down so the rotation is more easily controllable.



        if (RobotState.isSim()) {
            driveSubsystem.setDefaultCommand(
            driveSubsystem.driveArcade(
                    () -> driverController.getRawAxis(1) * DRIVE_SCALING,
                    () -> driverController.getRawAxis(0) * ROTATION_SCALING,
                    () -> driverController.getRightTriggerAxis() // im sorry idk what to put here arin
            ));
            //driveSubsystem.setDefaultCommand(driveSubsystem.driveAtTargetPose(new Pose2d(1,1,new Rotation2d())));
        }
        else{
            driveSubsystem.setDefaultCommand(
            driveSubsystem.driveArcade(
                    () -> driverController.getLeftY() * DRIVE_SCALING,
                    () -> driverController.getRightX() * ROTATION_SCALING,
                    () -> driverController.getRightTriggerAxis()));
        }


/*
            driveSubsystem.driveTank(
            () -> -operatorController.getLeftY() * DRIVE_SCALING - operatorController.getRightX() * ROTATION_SCALING,
            () -> -operatorController.getLeftY() * DRIVE_SCALING + operatorController.getRightX() * ROTATION_SCALING)); // make sure +- are correct
*/
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return autoChooser.getSelected();
    }



}