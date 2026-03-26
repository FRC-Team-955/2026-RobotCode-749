// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.localization.PoseSubsystem;

import static edu.wpi.first.wpilibj.drive.DifferentialDrive.arcadeDriveIK;
import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.RobotState.*;

@Logged
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
    DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(TRACK_WIDTH);

    double lSetPoint;
    double rSetPoint;

    public final PIDController leftPID = new PIDController(KP, KI, KD);
    public final PIDController rightPID = new PIDController(KP, KI, KD);

    private final TrapezoidProfile.Constraints forwardConstraints =
            new TrapezoidProfile.Constraints(
                    2.0,   // max velocity m/s TODO: TUNE THIS!!
                    0.5    // max acceleration (m/s)/s (ty mr buchanan)
            );
    private final ProfiledPIDController forwardPID =
            new ProfiledPIDController(1.95, 0.0, 1.63, forwardConstraints);
    double MOMENTUM_DAMPING =0.23; //Addional braking!


    // overall angle controller
    private final PIDController anglePID = new PIDController(2.05, 0.0, 0.18);

    double muffle = 100;

    private Timer timer = new Timer();

    /// TODO: TUNE THIS
    DifferentialDrivetrainSim drivetrainSim = new DifferentialDrivetrainSim(
            DCMotor.getNEO(2),       // 2 NEO motors on each side of the drivetrain.
            8.45,                    // Gearing
            1.0/12.0 * 35.0 * (Math.pow(DBASE_LENGTH_WITH_BUMPERS+INTAKE_EXTENSION_LENGTH,2) + Math.pow(DBASE_WIDTH_WITH_BUMPERS,2)), // rough estimate based on pathplanner formula (assumes uniform mass so definitely off)
            35.0,                    // 74ish lbs = 33.566 plus a little bit.
            Units.inchesToMeters(3), // The robot uses 3" radius wheels.
            TRACK_WIDTH,                  // what u think it is
            // The standard deviations for measurement noise:
            // x and y:          0.001 m
            // heading:          0.001 rad
            // l and r velocity: 0.1   m/s
            // l and r position: 0.005 m
            VecBuilder.fill(0.00001, 0.00001, 0.001, 0.01, 0.01, 0.005, 0.005)); //smaller number makes sim tweak out less



    /// ///// pathplanner
    RobotConfig configz;



    public CANDriveSubsystem(PoseSubsystem ps) {
        this.ps = ps;
        drivetrainSim.setPose(initialPose);

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


        try{
            configz = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }
        AutoBuilder.configure(
                this::getPose, // Robot pose supplier
                this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) -> driveRobotRelative(speeds, feedforwards), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
                new PPLTVController(
                        VecBuilder.fill(0.08, 0.08, 0.25),  //  state costs [x, y, heading]
                        VecBuilder.fill(0.55, 0.55),           //  input costs [left v, right v]    //// TODO: tune this
                        0.02
                ),
                configz, // The robot configuration
                () -> {
                    return false; // no flip path for opposite alliance; pose subsystem compensates internally.
                },
                this // Reference to this subsystem to set requirements
        );


    }
    public Pose2d getPose() {
        return globalPose;
    }
    public ChassisSpeeds getRobotRelativeSpeeds() {
        double leftMPS = leftLeader.getEncoder().getVelocity();
        double rightMPS = rightLeader.getEncoder().getVelocity();

        DifferentialDriveWheelSpeeds wheelSpeeds =
                new DifferentialDriveWheelSpeeds(leftMPS, rightMPS);


        return kinematics.toChassisSpeeds(wheelSpeeds);
    }
    public void driveRobotRelative(ChassisSpeeds speeds, DriveFeedforwards ff) {
        DifferentialDriveWheelSpeeds wheelSpeeds =
                kinematics.toWheelSpeeds(speeds);

        double leftTarget = wheelSpeeds.leftMetersPerSecond;
        double rightTarget = wheelSpeeds.rightMetersPerSecond;

        double leftVel = leftLeader.getEncoder().getVelocity();
        double rightVel = rightLeader.getEncoder().getVelocity();

        double leftOut = 0.05* leftPID.calculate(leftVel, leftTarget);  //TODO: tune this number
        double rightOut = 0.05 * rightPID.calculate(rightVel, rightTarget);

        leftOut = MathUtil.clamp(leftOut, -1, 1);
        rightOut = MathUtil.clamp(rightOut, -1, 1);

        drive.tankDrive(-leftOut, -rightOut);
    }


    int tempReadThrottle = 0;
    DifferentialDriveWheelSpeeds wheelSpeedsPeriodic =  new DifferentialDriveWheelSpeeds();
    @Override
    public void periodic() {
        tempReadThrottle++;
        //logging

        SmartDashboard.putNumber("llEncoderMeters", leftLeader.getEncoder().getPosition());
        SmartDashboard.putNumber("rlEncoderMeters", rightLeader.getEncoder().getPosition());
        SmartDashboard.putNumber("leftSetPoint", lSetPoint);
        SmartDashboard.putNumber("rightSetPoint", rSetPoint);
        SmartDashboard.putNumber("lPIDPoint", leftPID.getSetpoint());
        SmartDashboard.putNumber("rPIDPoint", rightPID.getSetpoint());
        SmartDashboard.putNumber("percentage", muffle);

        if(tempReadThrottle == 20) {
            highTempAlert.set((leftLeader.getMotorTemperature() > 50.0 || leftFollower.getMotorTemperature() > 50.0
                    || rightFollower.getMotorTemperature() > 50.0 || rightLeader.getMotorTemperature() > 50.0));
            SmartDashboard.putBoolean("highTempAlert", highTempAlert.get());
            tempReadThrottle =0;
        }
        ///  SIM AND NON-SIM GLOBAL VELOCITIES
        {

            // SparkMax encoder velocity is RPM by default
            double leftMPS = leftLeader.getEncoder().getVelocity();
            double rightMPS = rightLeader.getEncoder().getVelocity();




            wheelSpeedsPeriodic.leftMetersPerSecond = leftMPS;
            wheelSpeedsPeriodic.rightMetersPerSecond = rightMPS;

            ChassisSpeeds robotSpeeds =
                    kinematics.toChassisSpeeds(wheelSpeedsPeriodic);

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
            double percent = this.muffle;
            double limitedX = (percent/100)*limit.calculate(xSpeed.getAsDouble());
            double rot = (percent/200 + 0.5)*zRotation.getAsDouble();
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






    public void resetOdometry(Pose2d p){
        if(isSim()){
            drivetrainSim.setPose(p);
        }
        ps.resetOdometry(p);
    }
    public Command cresetOdometry(Pose2d p){
        return runOnce(()->resetOdometry(p));
    } //not run



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

        return (dist < 0.2) && (Math.abs(angleError) < Math.toRadians(5));
    }

    private boolean isAtPoseLessAccurate(Pose2d current, Pose2d target) {
        double dx = current.getX() - target.getX();
        double dy = current.getY() - target.getY();
        double dist = Math.hypot(dx, dy); // :)

        double angleError =
                current.getRotation().minus(target.getRotation()).getRadians();

        return (dist < 0.19) && (Math.abs(angleError) < Math.toRadians(7));
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

    public Command driveAtTargetPoseLessAccurate(Pose2d target){
        return run(()->funcDriveAtTargetPose(target)).until(()->isAtPoseLessAccurate(globalPose,target)).finallyDo(() -> drive.tankDrive(0, 0));
    }

    public Command driveAtTargetPoseSup(Supplier<Pose2d> x){
        return run(()->funcDriveAtTargetPose(x.get())).until(()->isSettledAtPose(globalPose,x.get())).finallyDo(() -> drive.tankDrive(0, 0));
    }

    public void simulationPeriodic() {


        // Set the inputs to the system. Note that we need to convert
        drivetrainSim.setInputs(-leftLeader.get() * RobotController.getInputVoltage(),
                -rightLeader.get() * RobotController.getInputVoltage());
        // Advance the model by 20 ms. Note that if you are running this
        // subsystem in a separate thread or have changed the nominal timestep
        // of TimedRobot, this value needs to match it.
        drivetrainSim.update(0.02);




        ps.resetOdometry(drivetrainSim.getPose());
        if(isSim()) {
            globalPose = drivetrainSim.getPose();
        }








        if(m_leftEncoderSim == null || m_rightEncoderSim==null){
            System.out.println("IDIOT. CALL ARIN AND TELL HIM TO GET A BRAIN");
        }
        else {
            m_leftEncoderSim.setPosition(-drivetrainSim.getLeftPositionMeters());
            m_leftEncoderSim.setVelocity(-drivetrainSim.getLeftVelocityMetersPerSecond());
            m_rightEncoderSim.setPosition(-drivetrainSim.getRightPositionMeters());
            m_rightEncoderSim.setVelocity(-drivetrainSim.getRightVelocityMetersPerSecond());
        }
    }

}