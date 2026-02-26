// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import choreo.trajectory.*; // dumb all include


import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LTVUnicycleController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.DSAndFieldUtil;

import static edu.wpi.first.wpilibj.drive.DifferentialDrive.arcadeDriveIK;
import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.DSAndFieldUtil.INITIAL_POSE;
import static frc.robot.DSAndFieldUtil.addPose;


public class CANDriveSubsystem extends SubsystemBase {
    private final SparkMax leftLeader;
    private final SparkMax leftFollower;
    private final SparkMax rightLeader;
    private final SparkMax rightFollower;

    private SparkRelativeEncoderSim m_leftEncoderSim;
    private SparkRelativeEncoderSim m_rightEncoderSim;

    private final PoseSubsystem ps; //Pose Estimator Class
    private final DifferentialDrive drive; //builtin wpilib drive
    private final LTVUnicycleController controller = new LTVUnicycleController(0.02);

    private final SlewRateLimiter limit = new SlewRateLimiter(10 * Constants.DriveConstants.SAFE_SPEED_CAP);
    DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(DBASE_WIDTH);
    double lSetPoint;
    double rSetPoint;


    DifferentialDrivetrainSim drivetrainSim = new DifferentialDrivetrainSim(
            DCMotor.getNEO(2),       // 2 NEO motors on each side of the drivetrain.
            8.45,                    //
            7.5,                     // MOI from CAD??
            30.0,                    // The mass of the robot is 70 kg.
            Units.inchesToMeters(3), // The robot uses 3" radius wheels.
            DBASE_WIDTH,                  //
            // The standard deviations for measurement noise:
            // x and y:          0.001 m
            // heading:          0.001 rad
            // l and r velocity: 0.1   m/s
            // l and r position: 0.005 m
            VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));



    public CANDriveSubsystem(PoseSubsystem ps) {
        this.ps = ps;
        // create brushed motors for drive
        leftLeader = new SparkMax(LEFT_LEADER_ID, MotorType.kBrushless);
        leftFollower = new SparkMax(LEFT_FOLLOWER_ID, MotorType.kBrushless);
        rightLeader = new SparkMax(RIGHT_LEADER_ID, MotorType.kBrushless);
        rightFollower = new SparkMax(RIGHT_FOLLOWER_ID, MotorType.kBrushless);

        m_leftEncoderSim = new SparkRelativeEncoderSim(leftLeader);
        m_rightEncoderSim = new SparkRelativeEncoderSim(rightLeader);


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
        SmartDashboard.putNumber("leftLeaderEncoder", leftLeader.getEncoder().getPosition());
        SmartDashboard.putNumber("llEncoderMeters", leftLeader.getEncoder().getPosition()/ENCODER_UNITS_PER_METER);
        SmartDashboard.putNumber("leftFollowerEncoder", leftFollower.getEncoder().getPosition()); // shouldn't be needed, just here to make sure (actually maybe it goes the opposite direction idk)
        SmartDashboard.putNumber("rightLeaderEncoder", rightLeader.getEncoder().getPosition());
        SmartDashboard.putNumber("rlEncoderMeters", rightLeader.getEncoder().getPosition()/ENCODER_UNITS_PER_METER);
        SmartDashboard.putNumber("rightFollowerEncoder", rightFollower.getEncoder().getPosition()); // shouldn't be needed, just here to make sure
        SmartDashboard.putNumber("leftSetPoint", lSetPoint);
        SmartDashboard.putNumber("rightSetPoint", rSetPoint);

    }

    public void logMotors(DoubleSupplier xSpeed, DoubleSupplier zRotation, boolean squareInputs) {
        double qxSpeed = MathUtil.applyDeadband(xSpeed.getAsDouble(), 0.02);
        double qzRotation = MathUtil.applyDeadband(zRotation.getAsDouble(), 0.02);

        var speeds = arcadeDriveIK(qxSpeed, qzRotation, squareInputs);

        drive.arcadeDrive(xSpeed.getAsDouble(), zRotation.getAsDouble());
        System.out.print("L: "); System.out.print(leftLeader.getAppliedOutput()); System.out.print(" R: ");System.out.println(rightLeader.getAppliedOutput());
        System.out.print("LF: "); System.out.print(leftFollower.getAppliedOutput()); System.out.print(" RF: ");System.out.println(rightFollower.getAppliedOutput());
    }

    // Command factory to create command to drive the robot with joystick inputs.
    public Command driveArcade(DoubleSupplier xSpeed, DoubleSupplier zRotation) {
        return run(() -> {
            double limitedX = limit.calculate(xSpeed.getAsDouble());
            double rot = zRotation.getAsDouble();

            updateSetPoints(limitedX + rot, limitedX - rot);
            drive.arcadeDrive(limitedX, rot);
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

    public Command resetPIDSetpoints(){
        return this.runOnce(() -> {
            lSetPoint = leftLeader.getEncoder().getPosition()/ENCODER_UNITS_PER_METER;
            rSetPoint = rightLeader.getEncoder().getPosition()/ENCODER_UNITS_PER_METER;
        });
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






    public void resetOdometry(Pose2d p){
        ps.resetOdometry(p);
    }


    public void funcDriveAtTargetPose(Pose2d targetPose) {
        if (targetPose == null) return;

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
                wheelSpeeds.leftMetersPerSecond/3 ,
                wheelSpeeds.rightMetersPerSecond/3
        );



    }

    private boolean isAtPose(Pose2d current, Pose2d target) {
        double dx = current.getX() - target.getX();
        double dy = current.getY() - target.getY();
        double dist = Math.hypot(dx, dy); // :)

        double angleError =
                current.getRotation().minus(target.getRotation()).getRadians();

        return dist < 0.05 && Math.abs(angleError) < Math.toRadians(3);
    }

    public Command driveAtTargetPose(Pose2d target){
        return run(()->funcDriveAtTargetPose(target)).until(()->isAtPose(ps.getPose(),target)).finallyDo(() -> drive.tankDrive(0, 0));
    }

    private int simOutTs = 100;
    private int counter=0;
    StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault()
            .getStructTopic("SimPose", Pose2d.struct).publish();


    public void simulationPeriodic() {
        counter++;

        // Set the inputs to the system. Note that we need to convert
        drivetrainSim.setInputs(leftLeader.get() * RobotController.getInputVoltage(),
                rightLeader.get() * RobotController.getInputVoltage());
        // Advance the model by 20 ms. Note that if you are running this
        // subsystem in a separate thread or have changed the nominal timestep
        // of TimedRobot, this value needs to match it.
        drivetrainSim.update(0.02);



        publisher.set(drivetrainSim.getPose());


        if(counter==simOutTs) {
            System.out.println("Left output: " + leftLeader.get());
            System.out.println("Right output: " + rightLeader.get());
            counter = 0;
        }


        if(m_leftEncoderSim == null || m_rightEncoderSim==null){
            System.out.println("IDIOT. CALL ARIN AND TELL HIM TO GET A BRAIN");
        }
        else {
            m_leftEncoderSim.setPosition(drivetrainSim.getLeftPositionMeters()*ENCODER_UNITS_PER_METER);
            m_leftEncoderSim.setVelocity(drivetrainSim.getLeftVelocityMetersPerSecond()*ENCODER_UNITS_PER_METER);
            m_rightEncoderSim.setPosition(drivetrainSim.getRightPositionMeters()*ENCODER_UNITS_PER_METER);
            m_rightEncoderSim.setVelocity(drivetrainSim.getRightVelocityMetersPerSecond()*ENCODER_UNITS_PER_METER);
        }
    }

}