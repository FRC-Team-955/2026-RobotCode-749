// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static int DEBUG = 0;


  public static final class DriveConstants {
      public static final double DBASE_WIDTH = Units.inchesToMeters(21.5); // meters
      public static final double DBASE_WIDTH_WITH_BUMPERS = Units.inchesToMeters(33.0);
      public static final double DBASE_LENGTH_WITH_BUMPERS = Units.inchesToMeters(32.8);
      public static final Translation2d DBASE_CENTER = new Translation2d(DBASE_LENGTH_WITH_BUMPERS/2, DBASE_WIDTH_WITH_BUMPERS/2);
              // going to remove this distance from the pose of the bump corners for siming autos
      
      public static final Translation2d LEFT_BUMP_CORNER = new Translation2d(4, 6.48); // 4 m, 6.48 m
      public static final Translation2d RIGHT_BUMP_CORNER = new Translation2d(4, 1.58) // x = 4 m, y = 1.58 m
              // these points are based on the corner of the field located by the outpost on each alliance***

      public static final Translation2d LEFT_BUMP_START = new Translation2d(LEFT_BUMP_CORNER.getMeasureX() - DBASE_CENTER.getMeasureX(),
                                                                                LEFT_BUMP_CORNER.getMeasureY() - DBASE_CENTER.getMeasureY());
      public static final Translation2d RIGHT_BUMP_START = new Translation2d(RIGHT_BUMP_CORNER.getMeasureX() - DBASE_CENTER.getMeasureX(),
                                                                                 RIGHT_BUMP_CORNER.getMeasureY() + DBASE_CENTER.getMeasureY());


    // Motor controller IDs for drivetrain motors
    public static final int LEFT_LEADER_ID = 2;
    public static final int LEFT_FOLLOWER_ID = 3;
    public static final int RIGHT_LEADER_ID = 5;
    public static final int RIGHT_FOLLOWER_ID = 4;
      public static final int PIGEON_ID = 13;
    // Encoder units per meter, hopefully is right (we are fairly certain)
    public static final double ENCODER_UNITS_PER_METER = (8.45)/(Math.PI*5.844*0.0254);
    // Cap for PID (max speed while using PID, must be between 0 and 1)
    public static final double PID_DRIVE_CAP = 0.6;
    // PID constant determines acceleration, higher value means higher acceleration
    // Want this value to be as high as possible without overshoot
    public static final double PID_CONSTANT = 1;
    public static final double KP = 4;
    public static final double KI = 0;
    public static final double KD = 0.6;
    // Current limit for drivetrain motors. 60A is a reasonable maximum to reduce
    // likelihood of tripping breakers or damaging CIM motors
    public static final int DRIVE_MOTOR_CURRENT_LIMIT = 60;
      // This determines how fast the robot will slow down, a smaller value will make the robot accel/decel slower (less jerk)
      public static final double SAFE_SPEED_CAP = 0.2;
  }

  public static final class FuelConstants {
      public static final double kP = 1;
    // Motor controller IDs for Fuel Mechanism motors
    public static final int FEEDER_MOTOR_ID = 6; // checked
    public static final int INTAKE_LAUNCHER_MOTOR_ID = 14;
    public static final int SHOOTER_WHEELS_MOTOR_ID = 11;

    // Current limit and nominal voltage for fuel mechanism motors.
    public static final int FEEDER_MOTOR_CURRENT_LIMIT = 55;


    // Voltage values for various fuel operations. These values may need to be tuned
    // based on exact robot construction.
    // See the Software Guide for tuning information
      public static final double INTAKE_SCALE = -1.00;
      public static final double LAUNCH_SCALE = 0.749; //set to -0.955 for real?
    public static final double INTAKING_FEEDER_VOLTAGE = 7*INTAKE_SCALE;
    public static final double INTAKING_INTAKE_VOLTAGE = 4*INTAKE_SCALE;
    public static final double LAUNCHING_FEEDER_VOLTAGE = (8)*LAUNCH_SCALE; //
    public static final double LAUNCHING_LAUNCHER_VOLTAGE = 12*LAUNCH_SCALE;
    public static final double FEEDER_SPIN_UP_VOLTAGE = (-6)*LAUNCH_SCALE; //
      public static final double OUTTAKING_INTAKE_VOLTAGE = 4*INTAKE_SCALE;
      public static final double SHOOTER_SPIN_UP_VOLTAGE = -10.2;
      public static final double SHOOTER_LAUNCH_VOLTAGE = -6.7;
      public static final double SHOOTER_WEAK_LAUNCH_VOLTAGE = -5.2;
      public static final double SHOOTER_STRONG_SPEED = 58;
      public static final double SHOOTER_WEAK_SPEED = 37;
      public static final double CORNER_HIT_VELOCITY = 70;
      public static final double SHOOTING_INTAKE_VOLTAGE = -7;

  }
    public static class ClimbConstants {
      public static final double KP = 0.1;
      public static final double KI = 0;
      public static final double KD = 0;
      public static final int CLIMBER_ID = 1;
      public static final int LIMITSWITCH_ID = 0; // idk its not plugged in yet
      public static final double GEAR_RATIO = 1/35;
      public static final double MAX_OUTPUT = 12;
      public static final double ENCODER_CAP = 104; // in encoder ticks, NEEDS to be tuned (im guessing 150 but its a wild guess)
        public static final double kP = 1.0; ////// THIS NEEDS TUNING!!!!!!!!!!!!!!!!!!!!



    }

  public static final class OperatorConstants {
    // Port constants for driver and operator controllers. These should match the
    // values in the Joystick tab of the Driver Station software
    public static final int DRIVER_CONTROLLER_PORT = 0; /// DRIVER has just drive
    public static final int OPERATOR_CONTROLLER_PORT = 1; /// OPERATOR everything else

    // This value is multiplied by the joystick value when driving the robot to
    // help avoid driving and turning too fast and being difficult to control
    public static final double DRIVE_SCALING = .749; //
    public static final double ROTATION_SCALING = .55;
  }
}
