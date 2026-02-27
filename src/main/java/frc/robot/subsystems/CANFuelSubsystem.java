// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DSAndFieldUtil;
import frc.robot.subsystems.shootersim.ShooterSim;

import java.util.ArrayList;

import static frc.robot.Constants.FuelConstants.*;
import static frc.robot.DSAndFieldUtil.isSim;

@SuppressWarnings("removal") //weird deprecation warning. As all programmers know, suppressing errors is better than fixing them
public class CANFuelSubsystem extends SubsystemBase {
  private final SparkMax feederRoller;
  private final SparkMax intakeLauncherRoller;
  private final TalonFX shooterWheels;



  ShooterSim SS = new ShooterSim();

    public void setBrakeMode() {
        shooterWheels.setNeutralMode(NeutralModeValue.Brake);
    }
    public void setCoastMode() {
        shooterWheels.setNeutralMode(NeutralModeValue.Coast);
    }
  /** Creates a new CANBallSubsystem. */
  public CANFuelSubsystem() {
    // create brushLESS motors for each of the motors on the launcher mechanism
    intakeLauncherRoller = new SparkMax(INTAKE_LAUNCHER_MOTOR_ID, MotorType.kBrushless);
    feederRoller = new SparkMax(FEEDER_MOTOR_ID, MotorType.kBrushless);
    shooterWheels = new TalonFX(SHOOTER_WHEELS_MOTOR_ID,"rio");



    // put default values for various fuel operations onto the dashboard
    // all methods in this subsystem pull their values from the dashbaord to allow
    // you to tune the values easily, and then replace the values in Constants.java
    // with your new values. For more information, see the Software Guide.
    SmartDashboard.putNumber("Intaking feeder roller value", INTAKING_FEEDER_VOLTAGE);
    SmartDashboard.putNumber("Intaking intake roller value", INTAKING_INTAKE_VOLTAGE);
    SmartDashboard.putNumber("Launching feeder roller value", LAUNCHING_FEEDER_VOLTAGE);
    SmartDashboard.putNumber("Launching launcher roller value", LAUNCHING_LAUNCHER_VOLTAGE);
    SmartDashboard.putNumber("Spin-up feeder roller value", FEEDER_SPIN_UP_VOLTAGE);

    // create the configuration for the feeder roller, set a current limit and apply
    // the config to the controller
    SparkMaxConfig feederConfig = new SparkMaxConfig();
    feederConfig.smartCurrentLimit(FEEDER_MOTOR_CURRENT_LIMIT);
    feederRoller.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // create the configuration for the launcher roller, set a current limit, set
    // the motor to inverted so that positive values are used for both intaking and
    // launching, and apply the config to the controller
    SparkMaxConfig launcherConfig = new SparkMaxConfig();
    launcherConfig.inverted(true);
    launcherConfig.smartCurrentLimit(LAUNCHER_MOTOR_CURRENT_LIMIT);
    intakeLauncherRoller.configure(launcherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // A method to set the rollers to values for intaking
  public void intake() {
    feederRoller.setVoltage(SmartDashboard.getNumber("Intaking feeder roller value", INTAKING_FEEDER_VOLTAGE));
    intakeLauncherRoller
        .setVoltage(SmartDashboard.getNumber("Intaking intake roller value", INTAKING_INTAKE_VOLTAGE));
  }

  // A method to set the rollers to values for ejecting fuel out the intake. Uses
  // the same values as intaking, but in the opposite direction.
  public void eject() {
    feederRoller
        .setVoltage(-1 * SmartDashboard.getNumber("Intaking feeder roller value", INTAKING_FEEDER_VOLTAGE));
    intakeLauncherRoller
        .setVoltage(-1 * SmartDashboard.getNumber("Intaking launcher roller value", INTAKING_INTAKE_VOLTAGE));
  }

  // A method to set the rollers to values for launching.
  public void launch() {
    feederRoller.setVoltage(SmartDashboard.getNumber("Launching feeder roller value", LAUNCHING_FEEDER_VOLTAGE));
    intakeLauncherRoller
        .setVoltage(SmartDashboard.getNumber("Launching launcher roller value", LAUNCHING_LAUNCHER_VOLTAGE));
      shooterWheels.setVoltage(SHOOTER_LAUNCH_VOLTAGE); ///brake mode makes this stop
      if(isSim()){
          System.out.println("ARIN IS NOT DUMB");
      }
  }

    public void weakLaunch() {
        feederRoller.setVoltage(SmartDashboard.getNumber("Launching feeder roller value", LAUNCHING_FEEDER_VOLTAGE));
        intakeLauncherRoller
                .setVoltage(SmartDashboard.getNumber("Launching launcher roller value", LAUNCHING_LAUNCHER_VOLTAGE));
        shooterWheels.setVoltage(SHOOTER_WEAK_LAUNCH_VOLTAGE); ///brake mode makes this stop
    }


  // A method to stop the rollers
  public void stop() {
    feederRoller.set(0);
    intakeLauncherRoller.set(0);
    shooterWheels.setVoltage(0);
  }

  // A method to spin up the launcher roller while spinning the feeder roller to
  // push Fuel away from the launcher
  public void spinUp(double voltage) {
    feederRoller
        .setVoltage(SmartDashboard.getNumber("Spin-up feeder roller value", FEEDER_SPIN_UP_VOLTAGE));
    intakeLauncherRoller
        .setVoltage(SmartDashboard.getNumber("Launching launcher roller value", LAUNCHING_LAUNCHER_VOLTAGE));
    shooterWheels.setVoltage(voltage);
  }

  // A command factory to turn the spinUp method into a command that requires this
  // subsystem
  public Command spinUpCommand() {
    return this.run(() -> spinUp(SHOOTER_SPIN_UP_VOLTAGE));
  }
    public Command spinUpWeakCommand() {
        return this.run(() -> spinUp(SHOOTER_SPIN_UP_VOLTAGE));
    }

  // A command factory to turn the launch method into a command that requires this
  // subsystem
  public Command launchCommand() {
    return this.run(() -> launch());
  }
    public Command weakLaunchCommand() {
        return this.run(() -> weakLaunch());
    }

  public boolean isAtSpeed(){
      return (((-shooterWheels.getVelocity().getValueAsDouble())- 58) > -1.2);
  }
    public boolean isAtWeakSpeed(){
        return (((-shooterWheels.getVelocity().getValueAsDouble())- 29) > -1.2);
    }


    StructArrayPublisher<Pose3d> arrayPublisher = NetworkTableInstance.getDefault()
            .getStructArrayTopic("PathShot", Pose3d.struct).publish();
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
      SmartDashboard.putNumber("Shooter Velocity", -shooterWheels.getVelocity().getValueAsDouble()); // 58 running, ~61(?) for spinup
      SmartDashboard.putNumber("Shooter Encoder", -shooterWheels.getPosition().getValueAsDouble());

      if(isSim()) {
          System.out.print("SS Sim Shot: ");
          System.out.println(-shooterWheels.getVelocity().getValueAsDouble());
      }
      ArrayList<Pose3d> a = SS.SimShot(-shooterWheels.getVelocity().getValueAsDouble()+DSAndFieldUtil.GLOBAL_POSE.getX(), DSAndFieldUtil.GLOBAL_POSE,0,0);
      arrayPublisher.set( a.toArray(new Pose3d[0]));
  }
}
