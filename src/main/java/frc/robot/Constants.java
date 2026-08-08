// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(11)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(14.5); 
  // Maximum speed of the robot in meters per second, used to limit acceleration.

  // List of robot start positions, blue first then red (order 1, 2, 3 obviously)
  public static final List<Pose2d> STARTING_POSITIONS = List.of(
    new Pose2d(4,     7.5,  new Rotation2d(0)),
    new Pose2d(3.55,  4.05, new Rotation2d(0)),
    new Pose2d(4,     0.6,  new Rotation2d(0)),
    new Pose2d(12.53, 0.6,  new Rotation2d(Math.PI)),
    new Pose2d(12.98, 4.05, new Rotation2d(Math.PI)),
    new Pose2d(12.53, 7.5,  new Rotation2d(Math.PI)));

  // List of default autos
  public static final List<String> AUTO_LIST = List.of(
    "Left Auto Blue",
    "Middle Auto Blue",
    "Right Auto Blue",
    "Left Auto Red",
    "Middle Auto Red",
    "Right Auto Red");

  public static final class DrivebaseConstants
  {
    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {
    public static final CommandXboxController driverXbox   = new CommandXboxController(0);
    public static final CommandXboxController operatorXbox = new CommandXboxController(1);

    public static final double DEADBAND         = 0.1;
    public static final double LEFT_Y_DEADBAND  = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    =   6;
  }

  public static class LightsConstants {
    public static int port   =   0;
    public static int length = 111;
    public static int C_length = 16;
    public static int four_length = 18;
    public static int C4_length = C_length + four_length;

    public static enum LightsType {
      ENDGAME,
      INTAKE,
      IDLE,
      DISABLED
    }

    //Now in RGB instead of GRB
    public static class Colors {
      public static int[] RED     = new int[] { 255,   0,   0 };
      public static int[] GREEN   = new int[] {   0, 255,   0 };
      public static int[] BLUE    = new int[] {   0,   0, 255 };
      public static int[] MAGENTA = new int[] { 255,   0, 255 };
      public static int[] YELLOW  = new int[] { 255, 255,   0 };  
      public static int[] WHITE   = new int[] { 248, 255, 131 };
    }
  }
}