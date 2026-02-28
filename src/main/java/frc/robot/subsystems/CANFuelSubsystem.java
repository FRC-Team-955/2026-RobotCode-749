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
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.subsystems.shootersim.ShooterSim;

import java.util.ArrayList;

import static frc.robot.Constants.FuelConstants.*;
import static frc.robot.RobotState.*;

@SuppressWarnings("removal") //weird deprecation warning. As all programmers know, suppressing errors is better than fixing them
public class CANFuelSubsystem extends SubsystemBase {
  private final SparkMax feederRoller;
  private final SparkMax intakeLauncherRoller;
  private final TalonFX shooterWheels;



  ShooterSim SS = new ShooterSim();
    ArrayList<Pose3d> targetList = new ArrayList<>();

    public void setBrakeMode() {
        shooterWheels.setNeutralMode(NeutralModeValue.Brake);
    }
    public void setCoastMode() {
        shooterWheels.setNeutralMode(NeutralModeValue.Coast);
    }
  /** Creates a new CANBallSubsystem. */
  public CANFuelSubsystem() {

      targetList.add(new Pose3d(4.01,4.03,1.8288,new Rotation3d()));
      targetList.add(new Pose3d(4.319,3.505,1.8288,new Rotation3d()));
      targetList.add(new Pose3d(4.928,3.508,1.8288,new Rotation3d()));
      targetList.add(new Pose3d(5.230,4.038,1.8288,new Rotation3d()));
      targetList.add(new Pose3d(4.922,4.565,1.8288,new Rotation3d()));
      targetList.add(new Pose3d(4.312,4.561,1.8288,new Rotation3d()));
      targetList.add(new Pose3d(4.01,4.03,1.8288,new Rotation3d()));


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
    StructArrayPublisher<Pose3d> tarrayPublisher = NetworkTableInstance.getDefault()
            .getStructArrayTopic("TargetOutline", Pose3d.struct).publish();
    int counter = 0;
    int calculateEvery = 12; //4x a second

  @Override
  public void periodic() {
      counter++;
    // This method will be called once per scheduler run
      SmartDashboard.putNumber("Shooter Velocity", -shooterWheels.getVelocity().getValueAsDouble()); // 58 running, ~61(?) for spinup
      SmartDashboard.putNumber("Shooter Encoder", -shooterWheels.getPosition().getValueAsDouble());
        boolean badPose = false;
      if(counter == calculateEvery) {
          if(SS.poseHit(GLOBAL_POSE,targetList)<0){
              badPose=true;
          }
          ArrayList<Pose3d> a;
          if (!isSim()) {
              a = SS.SimShot(Math.abs(shooterWheels.getVelocity().getValueAsDouble()), RobotState.GLOBAL_POSE, ROBOT_VX, ROBOT_VY);
              arrayPublisher.set(a.toArray(new Pose3d[0]));
          } else {
              double cV = SS.getShooterVel(GLOBAL_POSE, ROBOT_VX, ROBOT_VY, targetList);
              if (cV < 0) {
                  System.out.print("No shot can be made. Details: ");
                  a = new ArrayList<Pose3d>();
                  if(badPose) {
                      System.out.println("BAD LOCATION! The current robot (x,y) cannot hit a shot.");
                  }
                  else{
                      System.out.print("BAD ANGLE! Current angle: "); System.out.println(GLOBAL_POSE.getRotation().getRadians());System.out.print(" Angle to hub needed: ");System.out.println(SS.toFaceHub().getRadians());
                  }

              } else {
                  System.out.print("HIT! Target AngV: ");
                  System.out.println(cV);
                  a = SS.SimShot(cV, RobotState.GLOBAL_POSE, ROBOT_VX, ROBOT_VY);
              }
              arrayPublisher.set(a.toArray(new Pose3d[0]));

          }
          tarrayPublisher.set(targetList.toArray(new Pose3d[0]));
          counter = 0;
      }

  }
}
