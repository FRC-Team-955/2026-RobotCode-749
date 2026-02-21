// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



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
     public static final double DBASE_WIDTH = 0.55; // meters

    // Motor controller IDs for drivetrain motors
    public static final int LEFT_LEADER_ID = 2;
    public static final int LEFT_FOLLOWER_ID = 3;
    public static final int RIGHT_LEADER_ID = 5;
    public static final int RIGHT_FOLLOWER_ID = 4;
      public static final int PIGEON_ID = 13;
    // Encoder units per meter, hopefully is right (we are fairly certain)
    public static final double ENCODER_UNITS_PER_METER = (8.45)/(Math.PI*5.844*0.0254);
    // Cap for PID (max speed while using PID, must be between 0 and 1)
    public static final double PID_DRIVE_CAP = 0.8;
    // PID constant determines acceleration, higher value means higher acceleration
    // Want this value to be as high as possible without overshoot
    public static final double PID_CONSTANT = 3;
    // Current limit for drivetrain motors. 60A is a reasonable maximum to reduce
    // likelihood of tripping breakers or damaging CIM motors
    public static final int DRIVE_MOTOR_CURRENT_LIMIT = 60;
      // This determines how fast the robot will slow down, a smaller value will make the robot accel/decel slower (less jerk)
      public static final double SAFE_SPEED_CAP = 0.2;
  }

  public static final class FuelConstants {
    // Motor controller IDs for Fuel Mechanism motors
    public static final int FEEDER_MOTOR_ID = 6; // checked
    public static final int INTAKE_LAUNCHER_MOTOR_ID = 1;
    public static final int SHOOTER_WHEELS_MOTOR_ID = 11;

    // Current limit and nominal voltage for fuel mechanism motors.
    public static final int FEEDER_MOTOR_CURRENT_LIMIT = 55;
    public static final int LAUNCHER_MOTOR_CURRENT_LIMIT = 55;

    // Voltage values for various fuel operations. These values may need to be tuned
    // based on exact robot construction.
    // See the Software Guide for tuning information
      public static final double INTAKE_SCALE = -1.00;
      public static final double LAUNCH_SCALE = 0.749; //set to -0.955 for real?
    public static final double INTAKING_FEEDER_VOLTAGE = (12)*INTAKE_SCALE;
    public static final double INTAKING_INTAKE_VOLTAGE = -12*INTAKE_SCALE;
    public static final double LAUNCHING_FEEDER_VOLTAGE = (9)*LAUNCH_SCALE;
    public static final double LAUNCHING_LAUNCHER_VOLTAGE = 12*LAUNCH_SCALE;
    public static final double FEEDER_SPIN_UP_VOLTAGE = (-6)*LAUNCH_SCALE;
    public static final double SPIN_UP_SECONDS = 0.5;
      public static final double SHOOTER_SPIN_UP_VOLTAGE = -7.6;
      public static final double SHOOTER_LAUNCH_VOLTAGE = -6.7;
  }

  public static final class OperatorConstants {
    // Port constants for driver and operator controllers. These should match the
    // values in the Joystick tab of the Driver Station software
    public static final int DRIVER_CONTROLLER_PORT = 1;
    public static final int OPERATOR_CONTROLLER_PORT = 1;

    // This value is multiplied by the joystick value when driving the robot to
    // help avoid driving and turning too fast and being difficult to control
    public static final double DRIVE_SCALING = .67; /// <--------------------- LOOK ITS 67!
    public static final double ROTATION_SCALING = .62;
  }
}
