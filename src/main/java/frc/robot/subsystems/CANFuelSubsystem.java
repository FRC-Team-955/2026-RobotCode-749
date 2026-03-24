// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.shootersim.ShooterSim;

import java.util.ArrayList;
import java.util.function.DoubleSupplier;

import static frc.robot.Constants.FuelConstants.*;
import static frc.robot.RobotState.*;

@SuppressWarnings("removal") //weird deprecation warning. As all programmers know, suppressing errors is better than fixing them
public class CANFuelSubsystem extends SubsystemBase {
  private final SparkMax feederRoller;
  private final TalonFX intakeLauncherRoller;
  private final TalonFX shooterWheels;
  private boolean runFeederAutoAim = false;
  private double hitVelocity = -1;


  ShooterSim SS = new ShooterSim();

    Pose3d target;

    public void setBrakeMode() {
        shooterWheels.setNeutralMode(NeutralModeValue.Brake);
    }
    public void setCoastMode() {
        shooterWheels.setNeutralMode(NeutralModeValue.Coast);
    }
  /** Creates a new CANBallSubsystem. */
  public CANFuelSubsystem() {


      target = new Pose3d((4.01+5.23)/2, 4,1.8288, new Rotation3d());


    // create brushLESS motors for each of the motors on the launcher mechanism
    intakeLauncherRoller = new TalonFX(INTAKE_LAUNCHER_MOTOR_ID);
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
    SmartDashboard.putNumber("Shooting intake roller value", SHOOTING_INTAKE_VOLTAGE);

    // create the configuration for the feeder roller, set a current limit and apply
    // the config to the controller
    SparkMaxConfig feederConfig = new SparkMaxConfig();
    feederConfig.smartCurrentLimit(FEEDER_MOTOR_CURRENT_LIMIT);
    feederRoller.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // create the configuration for the launcher roller, set a current limit, set
    // the motor to inverted so that positive values are used for both intaking and
    // launching, and apply the config to the controller
    TalonFXConfiguration launcherConfig = new TalonFXConfiguration();
    intakeLauncherRoller.getConfigurator().apply(launcherConfig);


  }

  // A method to set the rollers to values for intaking
  public void intake() {
      if (RobotState.isSim()) {
          System.out.println("Intake running!");
      }
    feederRoller.setVoltage(SmartDashboard.getNumber("Intaking feeder roller value", INTAKING_FEEDER_VOLTAGE));
    intakeLauncherRoller
        .setVoltage(SmartDashboard.getNumber("Intaking intake roller value", INTAKING_INTAKE_VOLTAGE));
  }

  // A method to set the rollers to values for ejecting fuel out the intake. Uses
  // the same values as intaking, but in the opposite direction.
  public void eject() {
    feederRoller
        .setVoltage(-SmartDashboard.getNumber("Intaking feeder roller value", INTAKING_FEEDER_VOLTAGE));
    intakeLauncherRoller
        .setVoltage( -SmartDashboard.getNumber("Shooting intake roller value", OUTTAKING_INTAKE_VOLTAGE));
  }

  // A method to set the rollers to values for launching.
  public void launch(DoubleSupplier voltage) {
      shooterRatio =1;
    feederRoller.setVoltage(SmartDashboard.getNumber("Launching feeder roller value", LAUNCHING_FEEDER_VOLTAGE)); //
    intakeLauncherRoller
        .setVoltage(-SmartDashboard.getNumber("Launching launcher roller value", -LAUNCHING_LAUNCHER_VOLTAGE));
      shooterWheels.setVoltage(voltage.getAsDouble()); ///brake mode makes this stop
      if(isSim()){
          System.out.println("SIM SHOOTING!!");
      } // need to make it so this also triggers shake command
  }

  public void funcShootAtTarget(double overrideVelocity){
      shooterRatio=1;
      if(hitVelocity<0){
          return;
      }
      if(overrideVelocity !=0){
          hitVelocity=overrideVelocity;
          runFeederAutoAim=true;
      }
      SmartDashboard.putNumber("AutoAimAngV",hitVelocity);
      if(isSim()){
          System.out.print("Auto aim vel: "); System.out.println(hitVelocity);
      }

      if(runFeederAutoAim){
          feederRoller.set(LAUNCHING_FEEDER_VOLTAGE);
          intakeLauncherRoller
                  .setVoltage(-SmartDashboard.getNumber("Launching launcher roller value", LAUNCHING_LAUNCHER_VOLTAGE));
      }
      else{
          feederRoller.set(0);

      }


      double current = -shooterWheels.getVelocity().getValueAsDouble();
      double error = hitVelocity - current;

      double output = error * Constants.FuelConstants.kP;

      output = Math.max(
              -10.6,
              Math.min(10.6, output)
      );

      shooterWheels.set(-output);
  }
  public Command shootAtTarget(double overrideVelocity){
      return run(()->funcShootAtTarget(0));
  }

  // A method to stop the rollers
  public void stop() {
    feederRoller.set(0);
    intakeLauncherRoller.set(0);
    shooterWheels.setVoltage(0);
    shooterRatio=-1;
  }

  // A method to spin up the launcher roller while spinning the feeder roller to
  // push Fuel away from the launcher
  public void spinUp(double voltage) {
    feederRoller
        .setVoltage(SmartDashboard.getNumber("Spin-up feeder roller value", FEEDER_SPIN_UP_VOLTAGE)); //
    intakeLauncherRoller
        .setVoltage(-SmartDashboard.getNumber("Launching launcher roller value", LAUNCHING_LAUNCHER_VOLTAGE));
    shooterWheels.setVoltage(voltage);


  }

  // A command factory to turn the spinUp method into a command that requires this
  // subsystem
  public Command spinUpCommand() {
      return this.run(() -> spinUp(SHOOTER_SPIN_UP_VOLTAGE));
  }

  // A command factory to turn the launch method into a command that requires this
  // subsystem
  public Command launchCommand(DoubleSupplier voltage) {
      return this.run(() -> launch(voltage));
  }

  public boolean isAtSpeed(double speed){
      shooterRatio = (-shooterWheels.getVelocity().getValueAsDouble()/speed);
      if(shooterRatio>1){
          shooterRatio=-2;
      }
      return (((-shooterWheels.getVelocity().getValueAsDouble())- speed) > -1.2);
  }

  public double toFaceHub(){
      if(isSim()){
          System.out.println("Debugging angle:");
          System.out.println(SS.toFaceHub().getRadians());
      }
      return SS.toFaceHub().getRadians();
  }

    public double toFaceHub(Pose2d Current){
        if(isSim()){
            System.out.println("Debugging angle:");
            System.out.println(SS.toFaceHub(Current).getRadians());
        }
        return SS.toFaceHub(Current).getRadians();
    }

    public double toFaceHub(double xi, double yi){
        double x = 4.62017 - xi;
        double y  = 4.0345 - yi;
        return Math.atan2(y,x)+Math.PI;
    }



    StructPublisher<Pose3d> targetPublisher = NetworkTableInstance.getDefault()
            .getStructTopic("target", Pose3d.struct).publish();
    int counter = 0;
    int calculateEvery = 15; //3.3x a second


  @Override
  public void periodic() {

        SmartDashboard.putNumber("Shooter %", shooterRatio);
      counter++;
    // This method will be called once per scheduler run
      SmartDashboard.putNumber("Shooter Velocity", -shooterWheels.getVelocity().getValueAsDouble()); // 58 running, ~61(?) for spinup
      SmartDashboard.putNumber("Shooter Encoder", -shooterWheels.getPosition().getValueAsDouble());

      if(counter == calculateEvery) {


          double cV = SS.getShooterVel(globalPose,target);
          SmartDashboard.putNumber("shooter target vel",cV);
          boolean closeEnoughAngle = (Math.abs(MathUtil.angleModulus(SS.toFaceHub().getRadians() - MathUtil.angleModulus(globalPose.getRotation().getRadians()))) <Math.PI/36);
          double currentAngV = -shooterWheels.getVelocity().getValueAsDouble();
          if(isSim()){
              currentAngV=cV;
          }

            if(Math.abs(cV-currentAngV) < 2  && cV>0 ){
                //probably hits
                if( closeEnoughAngle) {
                    runFeederAutoAim = true;
                    System.out.println("Auto aim OK");
                }
                else{
                    System.out.println("BAD ANGLE TO HUB!");
                }
            }
            else{
                runFeederAutoAim=false;
                System.out.println("Can't hit from here!");
            }


          targetPublisher.set(target);
          counter = 0;
      }

  }
}
