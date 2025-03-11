// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.PIDConstants;


import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(14.5);
      // Maximum speed of the robot in meters per second, used to limit acceleration.

  public static final class AutonConstants
  {

   public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; 
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double LEFT_X_DEADBAND  = 0.1;
    public static final double LEFT_Y_DEADBAND  = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }
  public static class CustomConstants
  {


    // OUR CUSTOM ROBOT CONSTANTS BELOW HERE.....
    // OUR CUSTOM ROBOT CONSTANTS BELOW HERE.....
    // OUR CUSTOM ROBOT CONSTANTS BELOW HERE.....

    // Elevator Speed Constants
    public static double elevatorSpeed = 0.70;
    public static double elevatorSpeedSlow = 0.05;

    // Elevator Height Position Constants
    public static double elevatorPositionL4 = 2270.00;
    public static double elevatorPositionL3 = 1450.00;
    public static double elevatorPositionL2 = 700.00;
    public static double ElevatorHeightMin = 0.00;

    // Climber Speed Constants
    public static double climbSpeed = 0.90;


    // Primary Shooter Head Constants BELOW...
    // Primary Shooter Head Constants BELOW...
    // Primary Shooter Head Constants BELOW...

    // Head Speed Constants
    public static double primaryHeadSpeed = 0.35;
    public static double primaryHeadSpeedSlow = 0.10;

    // Intake Wheels Constant
    public static double feedMotorSpeed = 0.85;

    // Head Pivoting Angle Constants - MIN & MAX
    public static double baseAngle = 200.00;
    public static double headAngleL4 = baseAngle - 18;
    public static double headAngleL3 = baseAngle - 18;
    public static double headAngleL2 = baseAngle - 18;
    public static double headMaxOutAngle = baseAngle - 70;
    public static double headDisabledAngle = baseAngle - 10;

  }
}
