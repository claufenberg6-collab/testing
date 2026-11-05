// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class MechanismConstants {

  public static final int armexapmle = 1;
    
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
      public static final int kCoDriverControllerPort = 1;
  }

  public static final class DriveConstants {
  // Driving Parameters - Note that these are not the maximum capable speeds of
  // the robot, rather the allowed maximum speeds
  public static final double kMaxSpeedMetersPerSecond = 1.0;
  public static final double kMaxAngularSpeed = .4 * Math.PI; // radians per second
  // Chassis configuration
  // Distance between centers of right and left wheels on robot
  public static final double kTrackWidth = Units.inchesToMeters(23.0);
  // Distance between front and back wheels on robot
  public static final double kWheelBase = Units.inchesToMeters(23.0);
  public static final SwerveDriveKinematics kDriveKinematics =
      new SwerveDriveKinematics(
          new Translation2d(kWheelBase / 2, kTrackWidth / 2),
          new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
          new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
          new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));
  // Angular offsets of the modules relative to the chassis in radians
  public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
  public static final double kFrontRightChassisAngularOffset = 0;
  public static final double kBackLeftChassisAngularOffset = Math.PI;
  public static final double kBackRightChassisAngularOffset = Math.PI / 2;
  // SPARK MAX CAN IDs
  public static final int kFrontLeftDrivingCanId = 7;
  public static final int kRearLeftDrivingCanId = 8;
  public static final int kFrontRightDrivingCanId = 4;
  public static final int kRearRightDrivingCanId = 3;
  public static final int kFrontLeftTurningCanId = 6;
  public static final int kRearLeftTurningCanId = 9;
  public static final int kFrontRightTurningCanId = 5;
  public static final int kRearRightTurningCanId = 2;
  public static final boolean kGyroReversed = false;
  public static int kIMU_ID = 10;
}
public static final class NeoMotorConstants {
  public static final double kFreeSpeedRpm = 5676;
  public static final double kVortexKv = 565;   // rpm/V
}
public static final class ModuleConstants {
  // The MAXSwerve module can be configured with one of three pinion gears: 12T,
  // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
  // more teeth will result in a robot that drives faster).
  public static final int kDrivingMotorPinionTeeth = 14;
  // Calculations required for driving motor conversion factors and feed forward
  public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
  public static final double kWheelDiameterMeters = Units.inchesToMeters(3);
  public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
  // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
  // teeth on the bevel pinion
  public static final double kDrivingWheelBevelGearTeeth = 45.0;
  public static final double kDrivingWheelFirstStageSpurGearTeeth = 22.0;
  public static final double kDrivingMotorBevelPinionTeeth = 15.0;
  public static final double kDrivingMotorReduction = (kDrivingWheelBevelGearTeeth * kDrivingWheelFirstStageSpurGearTeeth)
      / (kDrivingMotorPinionTeeth * kDrivingMotorBevelPinionTeeth);
  public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
      / kDrivingMotorReduction;
}
public static final class OIConstants {
  public static final int kDriverControllerPort = 0;
  public static final int kCoDriverControllerPort = 1;
  public static final double kDriveDeadband = 0.075;
  public static final double kTriggerButtonThreshold = 0.2;
  public static final int kCoDriverControllerPor = 0;
}
}
