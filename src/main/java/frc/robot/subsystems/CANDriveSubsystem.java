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
import edu.wpi.first.math.filter.SlewRateLimiter;
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


    private final PoseSubsystem ps; //Pose Estimator Class
  private final DifferentialDrive drive; //builtin wpilib drive
    private final LTVUnicycleController controller = new LTVUnicycleController(0.02);

    private final SlewRateLimiter limit = new SlewRateLimiter(10 * Constants.DriveConstants.SAFE_SPEED_CAP);
    DifferentialDriveKinematics kinematics =
            new DifferentialDriveKinematics(Units.inchesToMeters(27.0)); //27 = drivebase width?????
  double lSetPoint;
  double rSetPoint;
    Pose2d targetPose = new Pose2d();
    boolean enableTargetPose = false;

  public CANDriveSubsystem(PoseSubsystem ps) {
      this.ps = ps;
    // create brushed motors for drive
    leftLeader = new SparkMax(LEFT_LEADER_ID, MotorType.kBrushless);
    leftFollower = new SparkMax(LEFT_FOLLOWER_ID, MotorType.kBrushless);
    rightLeader = new SparkMax(RIGHT_LEADER_ID, MotorType.kBrushless);
    rightFollower = new SparkMax(RIGHT_FOLLOWER_ID, MotorType.kBrushless);

    // set up differential drive class
    drive = new DifferentialDrive(leftLeader, rightLeader);
    targetPose = ps.getPose();



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
      SmartDashboard.putNumber("leftLeaderEncoder", leftLeader.getEncoder().getPosition());
      SmartDashboard.putNumber("leftFollowerEncoder", leftFollower.getEncoder().getPosition()); // shouldn't be needed, just here to make sure (actually maybe it goes the opposite direction idk)
      SmartDashboard.putNumber("rightLeaderEncoder", rightLeader.getEncoder().getPosition());
      SmartDashboard.putNumber("rightFollowerEncoder", rightFollower.getEncoder().getPosition()); // shouldn't be needed, just here to make sure
      SmartDashboard.putNumber("leftSetPoint", lSetPoint);
      SmartDashboard.putNumber("rightSetPoint", rSetPoint);


      driveAtTargetPose();

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
      return this.run(() -> {
              updateSetPoints(limit.calculate(xSpeed.getAsDouble()) + zRotation.getAsDouble(), limit.calculate(xSpeed.getAsDouble()) - zRotation.getAsDouble());
              drive.arcadeDrive(limit.calculate(xSpeed.getAsDouble()) ,zRotation.getAsDouble());
      });
  }



  public Command driveTank(DoubleSupplier leftSpeed, DoubleSupplier rightSpeed) {
      // setpoints included just for logging it
      // use setpoints to tune encoder units
      return new SequentialCommandGroup( // THIS WILL NOT WORK (look at drivearcade)
              run(() -> updateSetPoints(leftSpeed.getAsDouble(), rightSpeed.getAsDouble())),
              run(() -> drive.tankDrive(leftSpeed.getAsDouble(), rightSpeed.getAsDouble())));
  }
    private void updateSetPoints(double leftSpeed, double rightSpeed) {
        lSetPoint += leftSpeed;
        rSetPoint += rightSpeed;

    }
  
  public Command setPIDSetpoints(DoubleSupplier lPoint, DoubleSupplier rPoint) {
    return this.runOnce(() -> {
      updateSetPoints(lPoint.getAsDouble(), rPoint.getAsDouble());
    }
    );
  }

  public Command autoDrivePID() {
      return this.run(() -> {
          drive.tankDrive(MathUtil.clamp(PID_CONSTANT * (lSetPoint - leftLeader.getEncoder().getPosition()/ENCODER_UNITS_PER_METER), -1 * PID_DRIVE_CAP, PID_DRIVE_CAP),
                            MathUtil.clamp(PID_CONSTANT * (rSetPoint - rightLeader.getEncoder().getPosition()/ENCODER_UNITS_PER_METER), -1 * PID_DRIVE_CAP, PID_DRIVE_CAP));
      }).until(() -> {
        double lDiff = Math.abs(leftLeader.getEncoder().getPosition()/ENCODER_UNITS_PER_METER-lSetPoint);
        double rDiff = Math.abs(rightLeader.getEncoder().getPosition()/ENCODER_UNITS_PER_METER-rSetPoint);
        return lDiff < 0.1 && rDiff < 0.1;
      });
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
    public Command CresetOdometry(){
        return run(()->resetOdometry(new Pose2d()));
    }

    public Command goPathFollow(Optional<Trajectory<DifferentialSample>> trajectory, Timer timer){
      return run(()->goPath(trajectory, timer));
    }


    public Command setTargetPoint(Pose2d a){
      return run(()->{targetPose=a;enableTargetPose=true;});
    }

    public Command toggleUseTargetPoint(){
      return run(()->enableTargetPose=!enableTargetPose);
    }

    public void driveAtTargetPose(Pose2d targetPose) {
        if (!enableTargetPose || targetPose == null) return;

        Pose2d currentPose = ps.getPose();

        // LTV controller computes chassis speeds
        ChassisSpeeds speeds = controller.calculate(
                currentPose,
                targetPose,
                0.0, // desired linear velocity
                0.0  // desired angular velocity
        );

        // Convert to wheel speeds
        DifferentialDriveWheelSpeeds wheelSpeeds =
                kinematics.toWheelSpeeds(speeds);



        drive.tankDrive(
                wheelSpeeds.leftMetersPerSecond ,
                wheelSpeeds.rightMetersPerSecond
        );

        if (isAtPose(currentPose, targetPose)) {
            drive.tankDrive(0, 0);
            enableTargetPose = false;
        }


    }

    private boolean isAtPose(Pose2d current, Pose2d target) {
        double dx = current.getX() - target.getX();
        double dy = current.getY() - target.getY();
        double dist = Math.hypot(dx, dy); // :)

        double angleError =
                current.getRotation().minus(target.getRotation()).getRadians();

        return dist < 0.05 && Math.abs(angleError) < Math.toRadians(3);
    }

}
