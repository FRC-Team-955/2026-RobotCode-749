// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;


import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LTVUnicycleController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;

import static edu.wpi.first.wpilibj.drive.DifferentialDrive.arcadeDriveIK;
import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.RobotState.*;


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

    public final PIDController leftPID = new PIDController(1, 0, 0);
    public final PIDController rightPID = new PIDController(1, 0, 0);

/// TODO: TUNE THIS
    DifferentialDrivetrainSim drivetrainSim = new DifferentialDrivetrainSim(
            DCMotor.getNEO(2),       // 2 NEO motors on each side of the drivetrain.
            8.45,                    //
            4,                     // MOI from CAD??
            29.76,                    // The mass of the robot is not 30 kg.
            Units.inchesToMeters(3), // The robot uses 3" radius wheels.
            DBASE_WIDTH,                  // what u think it is
            // The standard deviations for measurement noise:
            // x and y:          0.001 m
            // heading:          0.001 rad
            // l and r velocity: 0.1   m/s
            // l and r position: 0.005 m
            VecBuilder.fill(0.00001, 0.00001, 0.001, 0.01, 0.01, 0.005, 0.005)); //smaller number makes sim tweak out less



    public CANDriveSubsystem(PoseSubsystem ps) {
        this.ps = ps;
        drivetrainSim.setPose(INITIAL_POSE);
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
        double wheelCircumference = 2 * Math.PI * Units.inchesToMeters(3);
        double gearRatio = 8.45;

        // Set configuration to follow each leader and then apply it to corresponding
        // follower. Resetting in case a new controller is swapped
        // in and persisting in case of a controller reset due to breaker trip
        config.follow(leftLeader);
        config.encoder.positionConversionFactor(
                wheelCircumference / gearRatio
        );

        config.encoder.velocityConversionFactor(
                wheelCircumference / gearRatio / 60.0
        );

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
        SmartDashboard.putNumber("llEncoderMeters", leftLeader.getEncoder().getPosition());
        SmartDashboard.putNumber("leftFollowerEncoder", leftFollower.getEncoder().getPosition()); // shouldn't be needed, just here to make sure (actually maybe it goes the opposite direction idk)
        SmartDashboard.putNumber("rightLeaderEncoder", rightLeader.getEncoder().getPosition());
        SmartDashboard.putNumber("rlEncoderMeters", rightLeader.getEncoder().getPosition());
        SmartDashboard.putNumber("rightFollowerEncoder", rightFollower.getEncoder().getPosition()); // shouldn't be needed, just here to make sure
        SmartDashboard.putNumber("leftSetPoint", lSetPoint);
        SmartDashboard.putNumber("rightSetPoint", rSetPoint);
        SmartDashboard.putNumber("lPIDPoint", leftPID.getSetpoint());
        SmartDashboard.putNumber("rPIDPoint", rightPID.getSetpoint());


        ///  SIM AND NON-SIM GLOBAL VELOCITIES
         {

            // SparkMax encoder velocity is RPM by default
            double leftMPS = leftLeader.getEncoder().getVelocity();
            double rightMPS = rightLeader.getEncoder().getVelocity();




            DifferentialDriveWheelSpeeds wheelSpeeds =
                    new DifferentialDriveWheelSpeeds(leftMPS, rightMPS);

            ChassisSpeeds robotSpeeds =
                    kinematics.toChassisSpeeds(wheelSpeeds);

            ChassisSpeeds fieldRelative =
                    ChassisSpeeds.fromRobotRelativeSpeeds(
                            robotSpeeds,
                            GLOBAL_POSE.getRotation()
                    );



            if(!isSim()){
                RobotState.ROBOT_VX = -fieldRelative.vxMetersPerSecond; //
                RobotState.ROBOT_VY = -fieldRelative.vyMetersPerSecond;
            }
            else{
                RobotState.ROBOT_VX = fieldRelative.vxMetersPerSecond; //
                RobotState.ROBOT_VY = fieldRelative.vyMetersPerSecond;
            }

        }


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

    private void updateSetPoints(double leftSpeed, double rightSpeed) {
        lSetPoint += leftSpeed;
        rSetPoint += rightSpeed;
        leftPID.setSetpoint(leftPID.getSetpoint()+leftSpeed);
        rightPID.setSetpoint(rightPID.getSetpoint()+rightSpeed);

    }

    public void resetSetPoints() {
        lSetPoint = leftLeader.getEncoder().getPosition();
        rSetPoint = rightLeader.getEncoder().getPosition();
        leftPID.setSetpoint(leftLeader.getEncoder().getPosition());
        leftPID.setTolerance(0.05);
        rightPID.setSetpoint(rightLeader.getEncoder().getPosition());
        rightPID.setTolerance(0.05);
    }

    public DoubleSupplier giveLeftEncoder() {
        return () -> leftLeader.getEncoder().getPosition();
    }

    public DoubleSupplier giveRightEncoder() {
        return () -> rightLeader.getEncoder().getPosition();
    }

    public Command setPIDSetpoints(DoubleSupplier lPoint, DoubleSupplier rPoint) {
        return this.runOnce(() -> {
                    updateSetPoints(lPoint.getAsDouble(), rPoint.getAsDouble());
                }
        );
    }

    public Command autoDrivePID(DoubleSupplier leftEncoder, DoubleSupplier rightEncoder) {
        return this.run(() -> {
            drive.tankDrive(MathUtil.clamp(leftPID.calculate(leftEncoder.getAsDouble()), -1 * PID_DRIVE_CAP, PID_DRIVE_CAP),
                    MathUtil.clamp(rightPID.calculate(rightEncoder.getAsDouble()), -1 * PID_DRIVE_CAP, PID_DRIVE_CAP));
        }).until(() -> {
            return Math.abs(leftPID.getError()) < 0.1 && Math.abs(rightPID.getError()) < 0.1;
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


    StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault()
            .getStructTopic("SimPose", Pose2d.struct).publish();


    public void simulationPeriodic() {


        // Set the inputs to the system. Note that we need to convert
        drivetrainSim.setInputs(leftLeader.get() * RobotController.getInputVoltage(),
                rightLeader.get() * RobotController.getInputVoltage());
        // Advance the model by 20 ms. Note that if you are running this
        // subsystem in a separate thread or have changed the nominal timestep
        // of TimedRobot, this value needs to match it.
        drivetrainSim.update(0.02);



        publisher.set(drivetrainSim.getPose());
        RobotState.GLOBAL_POSE = drivetrainSim.getPose();








        if(m_leftEncoderSim == null || m_rightEncoderSim==null){
            System.out.println("IDIOT. CALL ARIN AND TELL HIM TO GET A BRAIN");
        }
        else {
            m_leftEncoderSim.setPosition(drivetrainSim.getLeftPositionMeters());
            m_leftEncoderSim.setVelocity(drivetrainSim.getLeftVelocityMetersPerSecond());
            m_rightEncoderSim.setPosition(drivetrainSim.getRightPositionMeters());
            m_rightEncoderSim.setVelocity(drivetrainSim.getRightVelocityMetersPerSecond());
        }
    }

}