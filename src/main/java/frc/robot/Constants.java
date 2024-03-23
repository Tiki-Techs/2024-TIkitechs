// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.Shooter.State;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be
 * declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag

  public static final class AutonConstants {
    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
    public static final PIDConstants ANGLE_PID = new PIDConstants(0.4, 0, 0.01);
  }

  public static class LED {
    public static final int redAlliance = 3;
    public static final int blueAlliance = 4;
    public static final int Ready = 5;
  }

  public static final class ShooterConstants {
    public static final int m_LeaderID = 9;
    public static final int m_FollowerID = 10;

    public static final int m_IndexID = 15;

    public static final List<Entry<Double, State>> SHOOTER_MAP = Arrays.asList(
        Map.entry(0.0, new State(4000, 55.5)),
        Map.entry(1.59, new State(4000, 55.5)),
        Map.entry(1.66, new State(5700, 53.8)),
        Map.entry(1.83, new State(5700, 51)),
        Map.entry(2.08, new State(5700, 47.5)),
        Map.entry(2.5, new State(5700, 42.2)),
        Map.entry(2.86, new State(5700, 38)),
        Map.entry(3.14, new State(5700, 36)),
                Map.entry(3.5, new State(5700, 34)),

        Map.entry(3.86, new State(5700, 32.5)),
        Map.entry(3.9, new State(5700, 32.5)),
        Map.entry(6.0, new State(5700, 32.5)));
  }

  public static final class IntakeConstants {
    public static final int m_Intake = 11;
  }

  public static final class ClimbConstants {
    public static final int leadLimit = 0;
    public static final int followLimit = 1;

    public static final int leadMotor = 12;
    public static final int followMotor = 13;
  }

  public static final class HoodConstants {
    public static final int m_Lead = 14;
    public static final int m_Follow = 16;
    public static final double encoderOffset = -20.0;
    public static final double upperLimit = 66.0;
    public static final double lowerLimit = 32.0;

    public static final int EncoderID = 2;
  }

  public static final class DrivebaseConstants {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants {

    // Joystick Deadband
    public static final double LEFT_X_DEADBAND = 0.15;
    public static final double LEFT_Y_DEADBAND = 0.15;
    public static final double RIGHT_X_DEADBAND = 0.15;
    public static final double DEADBAND = 0.15;

    public static final double TURN_CONSTANT = 6;
  }
}
