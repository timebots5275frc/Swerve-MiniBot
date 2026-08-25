// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.customTypes.PID;
import frc.robot.customTypes.SwerveCanIDs;
import frc.robot.customTypes.SwerveModuleLocations;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants 
{

  //Operator Constants
  public static final class OperatorConstants {
    // Port constants for driver and operator controllers. These should match the
    // values in the Joystick tab of the Driver Station software
    public static final int JOYSTICK_PORT = 0;

    public static final int BUTTON_BOARD_PORT = 1;

    public static final int XBOX_CONTROLLER_PORT = 2;

    // This value is multiplied by the joystick value when rotating the robot to
    // help avoid turning too fast and beign difficult to control
    public static final double DRIVE_SCALING = .7;
    public static final double ROTATION_SCALING = .8;
  }
    
  public static class ButtonConstants
  {
  }

    public static final class DriveConstants {

      public static final int PIGEON_2_ID = 9;

      public static final SwerveCanIDs SwerveCanIDs = new SwerveCanIDs(
        2, // LEFT_FRONT_DRIVE_MOTOR_ID 
        1, // LEFT_FRONT_STEER_MOTOR_ID 
        3, // RIGHT_FRONT_DRIVE_MOTOR_ID 
        4, // RIGHT_FRONT_STEER_MOTOR_ID 
        5, // LEFT_REAR_DRIVE_MOTOR_ID 
        6, // LEFT_REAR_STEER_MOTOR_ID 
        8, // RIGHT_REAR_DRIVE_MOTOR_ID 
        7, // RIGHT_REAR_STEER_MOTOR_ID 
        10, // LEFT_FRONT_STEER_ENCODER_ID 
        11, // RIGHT_FRONT_STEER_ENCODER_ID 
        12, // LEFT_REAR_STEER_ENCODER_ID 
        13); // RIGHT_REAR_STEER_ENCODER_ID 

      public static final SwerveModuleLocations SwerveModuleLocations = new SwerveModuleLocations(
        ((29 / 2) - 3.25) * MathConstants.INCH_TO_METER, // LEFT_FRONT_WHEEL_X
        ((30 / 2) - 3.25) * MathConstants.INCH_TO_METER, // LEFT_FRONT_WHEEL_Y
        ((29 / 2) - 3.25) * MathConstants.INCH_TO_METER, // RIGHT_FRONT_WHEEL_X
        (-(30 / 2) + 3.25) * MathConstants.INCH_TO_METER, // RIGHT_FRONT_WHEEL_Y
        (-(29 / 2) + 3.25) * MathConstants.INCH_TO_METER, // RIGHT_REAR_WHEEL_X
        (-(30 / 2) + 3.25) * MathConstants.INCH_TO_METER, // RIGHT_REAR_WHEEL_Y
        (-(29 / 2) + 3.25) * MathConstants.INCH_TO_METER, // LEFT_REAR_WHEEL_X
        ((30 / 2) - 3.25) * MathConstants.INCH_TO_METER); //LEFT_REAR_WHEEL_Y

      public static final Translation2d LEFT_FRONT_MODULE_POSITION = new Translation2d(DriveConstants.SwerveModuleLocations.LEFT_FRONT_WHEEL_X, DriveConstants.SwerveModuleLocations.LEFT_FRONT_WHEEL_Y);
      public static final Translation2d RIGHT_FRONT_MODULE_POSITION = new Translation2d(DriveConstants.SwerveModuleLocations.RIGHT_FRONT_WHEEL_X, DriveConstants.SwerveModuleLocations.RIGHT_FRONT_WHEEL_Y);    
      public static final Translation2d RIGHT_REAR_MODULE_POSITION = new Translation2d(DriveConstants.SwerveModuleLocations.RIGHT_REAR_WHEEL_X, DriveConstants.SwerveModuleLocations.RIGHT_REAR_WHEEL_Y);
      public static final Translation2d LEFT_REAR_MODULE_POSITION = new Translation2d(DriveConstants.SwerveModuleLocations.LEFT_REAR_WHEEL_X, DriveConstants.SwerveModuleLocations.LEFT_REAR_WHEEL_Y);

      public static final double WHEEL_RADIUS = 2.0 * 0.0254; // meters * 0.98
      public static final double WHEEL_CIRCUMFERENCE = 2.0 * Math.PI * WHEEL_RADIUS; // meters/revolution

      public static final double MAX_DRIVE_SPEED = 3.5; // meters/second
      public static final double MAX_STEER_RATE = .5; // rotations/second of a wheel for steer.
      public static final double MAX_TWIST_RATE = .6 * 2.0 * Math.PI; // radians/second of the robot rotation.
      public static final double CONTROLLER_TWIST_RATE = 2; // constant turn rate for using controller

      // Drive motor gear ratio.
      // | Driving Gear | Driven Gear |
      // First Stage | 14 | 50 |
      // Second Stage | 28 | 16 |
      // Third Stage | 15 | 60 |
      //
      // Overall Gear Ratio = 0.1225
      // One rotation of the motor gives 0.1225 rotations of the wheel.
      // 8.163 rotations of the motor gives one rotation of the wheel.
      public static final double DRIVE_GEAR_RATIO = (14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 60.0);

      // Steer motor gear ratio
      // | Driving Gear | Driven Gear |
      // First Stage | 15 | 32 |
      // Second Stage | 10 | 40 |
      //
      // Overall Gear Ration = 0.1171875
      // One rotation of the motor gives 0.1171875 rotations of the wheel.
      // 8.533 rotations of the motor gives one rotation of the wheel.
      public static final double STEER_GEAR_RATIO = (15.0 / 32) * (10 / 40);

      public static final PID PID_Encoder_Steer = new PID(20, 10, 0);
      public static final PID PID_SparkMax_Steer = new PID(0.0001, 0, 0, 0, 0.00005);
      public static final PID PID_SparkMax_Drive = new PID(0.0003, 0, 0, 0, 0.00016);
  }
  public static final class MathConstants
  {
    public static final double INCH_TO_METER = 0.0254;
  }

    public static final class JoystickConstants
    {
      public static final double JOY_TRANSLATION_RATE_LIMIT = 6;
      public static final double JOY_TURN_RATE_LIMIT = 12;

      public static final double JOY_INPUT_VELOCITY_MULT = 1;
      public static final double JOY_INPUT_ROTATION_VELOCITY_MULT = 1;
    }

    public static final class ControllerConstants 
    {
      public static final int DRIVER_STICK_CHANNEL = 0;
      public static final int AUX_STICK_CHANNEL    = 1;
      public static final double DEADZONE_DRIVE    = 0.1;
      public static final double DEADZONE_STEER    = 0.3;
    }


}
