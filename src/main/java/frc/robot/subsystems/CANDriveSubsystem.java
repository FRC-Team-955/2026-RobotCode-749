// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
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
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.wpilibj.drive.DifferentialDrive.arcadeDriveIK;
import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.RobotState.*;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;
import org.ironmaple.simulation.drivesims.COTS;
import static edu.wpi.first.units.Units.*;


public class CANDriveSubsystem extends SubsystemBase {
    private final SparkMax leftLeader;
    private final SparkMax leftFollower;
    private final SparkMax rightLeader;
    private final SparkMax rightFollower;
    private final Alert highTempAlert = new Alert("Drive motors are too hot!", Alert.AlertType.kWarning);


    private SparkRelativeEncoderSim m_leftEncoderSim;
    private SparkRelativeEncoderSim m_rightEncoderSim;

    private final PoseSubsystem ps; //Pose Estimator Class
    private final DifferentialDrive drive; //builtin wpilib drive

    private final SlewRateLimiter limit = new SlewRateLimiter(10 * Constants.DriveConstants.SAFE_SPEED_CAP);
    DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(DBASE_WIDTH);

    double lSetPoint;
    double rSetPoint;

    public final PIDController leftPID = new PIDController(KP, KI, KD);
    public final PIDController rightPID = new PIDController(KP, KI, KD);

    private final TrapezoidProfile.Constraints forwardConstraints =
            new TrapezoidProfile.Constraints(
                    2.2,   // max velocity m/s TODO: TUNE THIS!!
                    0.5    // max acceleration (m/s)/s (ty mr buchanan)
            );
    private final ProfiledPIDController forwardPID =
            new ProfiledPIDController(1.6, 0.0, 1.63, forwardConstraints);
    double MOMENTUM_DAMPING =0.28; //Addional braking!


    // overall angle controller
    private final PIDController anglePID = new PIDController(2.05, 0.0, 0.18);

    double muffle = 100;

    private Timer timer = new Timer();
    private final SwerveDriveSimulation drivetrainSim;         // maple-sim physics body
    private final SelfControlledSwerveDriveSimulation simDrive; // high-level controller

    private SimulatedMotorController.GenericMotorController simLeftMotor;
    private SimulatedMotorController.GenericMotorController simRightMotor;


    private double simFwd = 0.0;
    private double simRot = 0.0;


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

        //INIT poseSubsystem
        ps.setSource(()->leftLeader.getEncoder().getPosition(), ()->rightLeader.getEncoder().getPosition());

        forwardPID.setTolerance(0.067);        // within 6.7 cm
        anglePID.setTolerance(Math.toRadians(2));
        anglePID.enableContinuousInput(-Math.PI, Math.PI); // fixes angle wrap! finally


        DriveTrainSimulationConfig simConfig = DriveTrainSimulationConfig.Default()
                .withGyro(COTS.ofPigeon2())
                .withSwerveModule(new SwerveModuleSimulationConfig(
                        DCMotor.getNEO(2),    // 2 NEOs per side (drive)
                        DCMotor.getNEO(1),    // steer : ignored for differential, use NEO as placeholder
                        8.45,                 // drive gear ratio
                        1.0,                  // steer gear ratio (unused)
                        Volts.of(0.1),        // drive friction voltage
                        Volts.of(0.1),        // steer friction voltage
                        Inches.of(3),         // wheel radius
                        KilogramSquareMeters.of(0.01),
                        1.0))                 // wheel COF (tune to match traction)
                .withTrackLengthTrackWidth(
                        Meters.of(DBASE_WIDTH),  // length  use track width as approx
                        Meters.of(DBASE_WIDTH))
                .withBumperSize(Inches.of(32.8), Inches.of(32.8))  // adjust to your bumpers
                .withRobotMass(Kilograms.of(35));

        drivetrainSim = new SwerveDriveSimulation(simConfig, initialPose);
        simDrive = new SelfControlledSwerveDriveSimulation(drivetrainSim);



// Register to the maple-sim world : this is what replaces sim update i think
        SimulatedArena.getInstance().addDriveTrainSimulation(drivetrainSim);


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
        SmartDashboard.putNumber("percentage", muffle);


        highTempAlert.set((leftLeader.getMotorTemperature() > 50.0 || leftFollower.getMotorTemperature() > 50.0
                || rightFollower.getMotorTemperature() > 50.0 || rightLeader.getMotorTemperature() > 50.0));
        SmartDashboard.putBoolean("highTempAlert", highTempAlert.get());
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
                            globalPose.getRotation()
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

    public void startTimer() {
        timer.reset();
        timer.start();
    }

    public void shake() {
        double shake = 0.6*(0.5-Math.round(MathUtil.inputModulus(timer.get()*5, 0, 1)));
        drive.arcadeDrive(shake, 0); // 2x per sec
    } // change the 0.3 to make the magnitude larger (how fast the motors run)
    // change the 5 in timer.get*5 to increase frequency (how often/fast the robot shakes)

    public void stopShake() {
        drive.arcadeDrive(0,0);
    }

    public void endTimer() {
        timer.stop();
    }

    public void increaseSens() {
        muffle += 5;
        if (muffle > 120) {
            muffle = 120;
        }
    }

    public void decreaseSens() {
        muffle -= 5;
        if (muffle < 5) {
            muffle = 5;
        }
    }

    public void sens100() {
        muffle = 100;
    }

    public void sens50() {
        muffle = 50;
    }

    // Command factory to create command to drive the robot with joystick inputs.
    public Command driveArcade(DoubleSupplier xSpeed, DoubleSupplier zRotation, BooleanSupplier shake) {
        return run(() -> {
            DoubleSupplier percent = () -> this.muffle;
            double limitedX = (percent.getAsDouble()/100)*limit.calculate(xSpeed.getAsDouble());
            double rot = (percent.getAsDouble()/200 + 0.5)*zRotation.getAsDouble();
            double isShake;
            double shakeSpeed = 0;
            if (shake.getAsBoolean()) {
                isShake = 1;
                shakeSpeed = 0.6*(0.5-Math.round(MathUtil.inputModulus(timer.get()*5, 0, 1)));
            } else {
                isShake = 0;
            }

            updateSetPoints(limitedX + rot, limitedX - rot);
            drive.arcadeDrive(limitedX + isShake*shakeSpeed, rot);
            simFwd = limitedX + isShake*shakeSpeed; //// why is it zero??
            simRot = rot;

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

    public Command autoSlowDrivePID(DoubleSupplier leftEncoder, DoubleSupplier rightEncoder) {
        return this.run(() -> {
            drive.tankDrive(MathUtil.clamp(leftPID.calculate(leftEncoder.getAsDouble()), -0.8 * PID_DRIVE_CAP, 0.8*PID_DRIVE_CAP),
                    MathUtil.clamp(rightPID.calculate(rightEncoder.getAsDouble()), -0.8 * PID_DRIVE_CAP, 0.8*PID_DRIVE_CAP));
                }
        ).until(() -> {
            return Math.abs(leftPID.getError()) < 0.1 && Math.abs(rightPID.getError()) < 0.1;
        });
    }






    public void resetOdometry(Pose2d p) {
        if (isSim()) {
            simDrive.setSimulationWorldPose(p);
            simDrive.resetOdometry(p);
        }
        ps.resetOdometry(p);
    }
    public Command cresetOdometry(Pose2d p){
        return run(()->resetOdometry(p));
    }



    StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault()
            .getStructTopic("TargetPose", Pose2d.struct).publish();
    public void funcDriveAtTargetPose(Pose2d targetPose) {
        if (targetPose == null) {
            drive.arcadeDrive(0, 0);
            return;
        }

        // Publish the target for dashboard / logging
        publisher.set(targetPose);

        Pose2d current = globalPose;

        // Compute vector to target
        double dx = targetPose.getX() - current.getX();
        double dy = targetPose.getY() - current.getY();
        double distance = Math.hypot(dx, dy);

        double currentAngle = current.getRotation().getRadians();
        double approachAngle = Math.atan2(dy, dx);       // angle to drive toward target
        double finalAngle    = targetPose.getRotation().getRadians(); // desired final orientation

        // Compute heading difference
        double headingError = MathUtil.angleModulus(approachAngle - currentAngle);

        // Alignment scale reduces forward speed if misaligned
        double alignmentScale = MathUtil.clamp(1.0 - Math.abs(headingError) / (Math.PI/2), 0.2, 1.0);

        // Compute forward speed using distance PID
        double forwardOutput = forwardPID.calculate(0, distance);
        // Correct sign so forward is toward the target
        double forward = forwardOutput * alignmentScale * Math.cos(headingError); //kinda works lol
        double velocity = Math.hypot(RobotState.ROBOT_VX, RobotState.ROBOT_VY);

        forward -= MOMENTUM_DAMPING * velocity * 1.0/distance; //damping??

        // Clamp forward to reasonable values
        forward = MathUtil.clamp(forward, -1.0, 1.0);

        // Turn toward target / final heading
        double turn;
        if (distance < 0.15) {
            // Near target  rotate to final orientation
            double finalHeadingError = MathUtil.angleModulus(finalAngle - currentAngle);
            turn = anglePID.calculate(0, finalHeadingError);
            turn = MathUtil.clamp(turn, -0.45, 0.45);

            // Stop forward motion
            forward = 0.0;
        } else {

            // Far from target  rotate toward target
            turn = anglePID.calculate(0, headingError);
            turn = MathUtil.clamp(turn, -0.6, 0.6);
        }


        if (distance < 0.5) {
            if(forward>0) {
                forward = Math.min(forward, distance + 0.09);
            }
            else{
                forward = Math.max(forward, distance - 0.09);
            }
        }


        drive.arcadeDrive(-forward, -turn);
    }

    private boolean isAtPose(Pose2d current, Pose2d target) {
        double dx = current.getX() - target.getX();
        double dy = current.getY() - target.getY();
        double dist = Math.hypot(dx, dy); // :)

        double angleError =
                current.getRotation().minus(target.getRotation()).getRadians();

        return (dist < 0.13) && (Math.abs(angleError) < Math.toRadians(5));
    }

    private int atPoseTicks = 0;
    private final int REQUIRED_TICKS = 8; //

    private boolean isSettledAtPose(Pose2d current, Pose2d target){
        if (isAtPose(current, target)) {
            atPoseTicks++;
            return atPoseTicks >= REQUIRED_TICKS;
        } else {
            atPoseTicks = 0;
            return false;
        }
    }

    public Command driveAtTargetPose(Pose2d target){
        return run(()->funcDriveAtTargetPose(target)).until(()->isSettledAtPose(globalPose,target)).finallyDo(() -> drive.tankDrive(0, 0));
    }

    @Override
    public void simulationPeriodic() {
        System.out.print("FWD: "); System.out.println(simFwd);
        var out = DifferentialDrive.arcadeDriveIK(simFwd, simRot, false); //simFwd, and simRot
        double leftMPS  = out.left  * 3.2;
        double rightMPS = out.right * 3.2;
        ChassisSpeeds speeds = kinematics.toChassisSpeeds(
                new DifferentialDriveWheelSpeeds(leftMPS, rightMPS));


        simDrive.runChassisSpeeds(speeds, new Translation2d(), false, true);

        SimulatedArena.getInstance().simulationPeriodic();
        simDrive.periodic();

        Pose2d simPose = simDrive.getActualPoseInSimulationWorld();
        ps.resetOdometry(simPose);
        if (isSim()) globalPose = simPose;

        double wheelRadius = Units.inchesToMeters(3);

        double leftPos  = drivetrainSim.getModules()[0].getDriveWheelFinalPosition().in(Radians) * wheelRadius;
        double leftVel  = drivetrainSim.getModules()[0].getDriveWheelFinalSpeed().in(RadiansPerSecond) * wheelRadius;
        double rightPos = drivetrainSim.getModules()[1].getDriveWheelFinalPosition().in(Radians) * wheelRadius;
        double rightVel = drivetrainSim.getModules()[1].getDriveWheelFinalSpeed().in(RadiansPerSecond) * wheelRadius;

        m_leftEncoderSim.setPosition(leftPos);
        m_leftEncoderSim.setVelocity(leftVel);
        m_rightEncoderSim.setPosition(rightPos);
        m_rightEncoderSim.setVelocity(rightVel);
    }

}