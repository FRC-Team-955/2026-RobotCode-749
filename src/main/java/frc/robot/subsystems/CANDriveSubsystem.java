// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import choreo.trajectory.*; // dumb all include

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.LTVUnicycleController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static edu.wpi.first.wpilibj.drive.DifferentialDrive.arcadeDriveIK;
import static frc.robot.Constants.DriveConstants.*;


public class CANDriveSubsystem extends SubsystemBase {
  private final SparkMax leftLeader;
  private final SparkMax leftFollower;
  private final SparkMax rightLeader;
  private final SparkMax rightFollower;


    private final PoseSubsystem ps = new PoseSubsystem(); //Pose Estimator Class
  private final DifferentialDrive drive; //builtin wpilib drive
    private final LTVUnicycleController controller = new LTVUnicycleController(0.02);

  double lSetPoint;
  double rSetPoint;

  public CANDriveSubsystem() {
    // create brushed motors for drive
    leftLeader = new SparkMax(LEFT_LEADER_ID, MotorType.kBrushless);
    leftFollower = new SparkMax(LEFT_FOLLOWER_ID, MotorType.kBrushless);
    rightLeader = new SparkMax(RIGHT_LEADER_ID, MotorType.kBrushless);
    rightFollower = new SparkMax(RIGHT_FOLLOWER_ID, MotorType.kBrushless);

    // set up differential drive class
    drive = new DifferentialDrive(leftLeader, rightLeader);



    // Set can timeout. Because this project only sets parameters once on
    // construction, the timeout can be long without blocking robot operation. Code
    // which sets or gets parameters during operation may need a shorter timeout.
    leftLeader.setCANTimeout(250);
    rightLeader.setCANTimeout(250);
    leftFollower.setCANTimeout(250);
    rightFollower.setCANTimeout(250);

    // Create the configuration to apply to motors. Voltage compensation
    // helps the robot perform more similarly on different
    // battery voltages (at the cost of a little bit of top speed on a fully charged
    // battery). The current limit helps prevent tripping
    // breakers.
    SparkMaxConfig config = new SparkMaxConfig();
    config.voltageCompensation(12);
    config.smartCurrentLimit(DRIVE_MOTOR_CURRENT_LIMIT);


    // Set configuration to follow each leader and then apply it to corresponding
    // follower. Resetting in case a new controller is swapped
    // in and persisting in case of a controller reset due to breaker trip
    config.follow(leftLeader);

    leftFollower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    config.follow(rightLeader);
    rightFollower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Remove following, then apply config to right leader
    config.disableFollowerMode();
    rightLeader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // Set config to inverted and then apply to left leader. Set Left side inverted
    // so that postive values drive both sides forward
    config.inverted(true);
    leftLeader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    //INIT ARINS CODE
      ps.setSource(()->leftLeader.getEncoder().getPosition(), ()->rightLeader.getEncoder().getPosition());
  }

  @Override
  public void periodic() {
      //logging
      SmartDashboard.putNumber("leftLeaderEncoder", leftLeader.getEncoder().getPosition()/ENCODER_UNITS_PER_METER);
      SmartDashboard.putNumber("leftFollowerEncoder", leftFollower.getEncoder().getPosition()/ENCODER_UNITS_PER_METER); // shouldn't be needed, just here to make sure (actually maybe it goes the opposite direction idk)
      SmartDashboard.putNumber("rightLeaderEncoder", rightLeader.getEncoder().getPosition()/ENCODER_UNITS_PER_METER);
      SmartDashboard.putNumber("rightFollowerEncoder", rightFollower.getEncoder().getPosition()/ENCODER_UNITS_PER_METER); // shouldn't be needed, just here to make sure
      SmartDashboard.putNumber("leftSetPoint", lSetPoint);
      SmartDashboard.putNumber("rightSetPoint", rSetPoint);
  }

  public void logMotors(DoubleSupplier xSpeed, DoubleSupplier zRotation, boolean squareInputs) {
      double qxSpeed = MathUtil.applyDeadband(xSpeed.getAsDouble(), 0.02);
      double qzRotation = MathUtil.applyDeadband(zRotation.getAsDouble(), 0.02);

      var speeds = arcadeDriveIK(qxSpeed, qzRotation, squareInputs);

      double m_leftOutput = speeds.left;
      double m_rightOutput = speeds.right;
      drive.arcadeDrive(xSpeed.getAsDouble(), zRotation.getAsDouble());
      System.out.print("L: "); System.out.print(leftLeader.getAppliedOutput()); System.out.print(" R: ");System.out.println(rightLeader.getAppliedOutput());
      System.out.print("LF: "); System.out.print(leftFollower.getAppliedOutput()); System.out.print(" RF: ");System.out.println(rightFollower.getAppliedOutput());
  }

  // Command factory to create command to drive the robot with joystick inputs.
  public Command driveArcade(DoubleSupplier xSpeed, DoubleSupplier zRotation) {
    return this.run(
        () -> logMotors(xSpeed,zRotation, false));
  }



  public Command driveTank(DoubleSupplier leftSpeed, DoubleSupplier rightSpeed) {
      // setpoints included just for logging it
      // use setpoints to tune encoder units
      lSetPoint += leftSpeed.getAsDouble(); //JIN THIS DOESNT RUN....... read about commands
      rSetPoint += rightSpeed.getAsDouble();

      return new SequentialCommandGroup(
              run(() -> drive.tankDrive(leftSpeed.getAsDouble(), rightSpeed.getAsDouble())), run(()->System.out.println("DEBUG"))  );
  }

  public Command drivePID(DoubleSupplier leftSpeed, DoubleSupplier rightSpeed) {
      // 2^12 = 4096 encoder units per revolution
      lSetPoint += leftSpeed.getAsDouble();
      rSetPoint += rightSpeed.getAsDouble();
      return this.run(
            () -> drive.tankDrive(MathUtil.clamp(3 * (lSetPoint - leftLeader.getEncoder().getPosition()/ENCODER_UNITS_PER_METER), -0.8, 0.8), MathUtil.clamp(3 * (rSetPoint - rightLeader.getEncoder().getPosition()/ENCODER_UNITS_PER_METER), -0.8, 0.8))
    );
  }



    public void followTrajectory(Optional<DifferentialSample> samples) {
        // Get the current pose of the robot
        DifferentialSample sample = samples.orElse(new choreo.trajectory.DifferentialSample(0,0,0,0,0,0,0,0,0,0,0,0));

        // Get the velocity feedforward specified by the sample
        ChassisSpeeds ff = sample.getChassisSpeeds();

        // Generate the next speeds for the robot
        ChassisSpeeds speeds = controller.calculate(
                ps.getPose(),
                sample.getPose(),
                ff.vxMetersPerSecond,
                ff.omegaRadiansPerSecond
        );


        DifferentialDriveKinematics kinematics =
                new DifferentialDriveKinematics(Units.inchesToMeters(27.0)); //27 = drivebase width?????

        // Apply the generated speeds
        DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds); //
        drive.tankDrive(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);
    }

    private boolean isRedAlliance() {
        return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue).equals(DriverStation.Alliance.Red);
    }
    public void goPath(Optional<Trajectory<DifferentialSample>> trajectory, Timer timer){
        if (trajectory.isPresent()) {
            // Sample the trajectory at the current time into the autonomous period
            Optional<DifferentialSample> sample = trajectory.get().sampleAt(timer.get(), isRedAlliance());

            if (sample.isPresent()) {
                this.followTrajectory(sample);
            }
        }

    }

    public void resetOdometry(Pose2d p){
      ps.resetOdometry(p);
    }

    public Command goPathFollow(Optional<Trajectory<DifferentialSample>> trajectory, Timer timer){
      return run(()->goPath(trajectory, timer));
    }
}
