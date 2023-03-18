// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second
    public static final double kDirectionSlewRate = 1.2;
    public static final double kMagnitudeSlewRate = 1.8;
    public static final double kRotationalSlewRate = 2.0;

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(18.5625);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(32.5625);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
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
    public static final int kFrontLeftDrivingCanId = 11;
    public static final int kRearLeftDrivingCanId = 13;
    public static final int kFrontRightDrivingCanId = 15;
    public static final int kRearRightDrivingCanId = 16;

    public static final int kFrontLeftTurningCanId = 10;
    public static final int kRearLeftTurningCanId = 12;
    public static final int kFrontRightTurningCanId = 14;
    public static final int kRearRightTurningCanId = 17;

    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 13;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;
     // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = .5; //subject to change
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static final class OIConstants {
    //public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;
    public static final double kMagnitudeDeadband = 0.05;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kIntakeControllerPort = 1;
  }
  //PWM's for wiring team
//public static final int LEFT_FRONT = 5;
//public static final int RIGHT_FRONT = 8;
//public static final int LEFT_BACK = 1;
//public static final int RIGHT_BACK = 3;
public static final int SHOOTER = 4;
public static final int INTAKE = 7;


public static double kDefaultAuto = 5.0;
//Analog inputs
public static final int RANGE_FINDER = 0;



// Axis
public static final int XBOX_LEFT_Y_AXIS = 1;
public static final int XBOX_LEFT_X_AXIS = 0;
public static final int RIGHT_TRIGGER = 3;
public static final double DRIVETRAINSPEED = 1;
public static final double AUTONOMOUS_SPEED = 0.2;
public static final double DRIVE_FORWARD_TIME = 3.5;
public static final int JOYSTICK_NUMBER = 0;
public static final double SHOOTER_SPEED = .5;
public static final double INTAKE_SPEED = .25;
public static final int CAMERA_RES_X = 320;
public static final int CAMERA_RES_Y = 240;
public static final double AUTO_SHOOT_TIME = 2.0;
public static final double SETPOINT_FORWARD = 1.5;

public static final class Intake {
  public static double fineGrainDistance = .25;
  public static final class A {
    public static double Position_LOW = -102; //subject to change
    public static double Position_HIGH = 87;
    public static final double kPlow = 0.01;
    public static final double kIlow = 0;
    public static final double kDlow = 0;
    public static final double kPhigh = .02;
    public static final double kIhigh = 0;
    public static final double kDhigh = 0;
  }
  public static final class B {
    public static double Position_LOW = -127;
    public static double Position_HIGH = 115;
    public static final double kPlow = 0.01;
    public static final double kIlow = 0;
    public static final double kDlow = 0;
    public static final double kPhigh = .02;
    public static final double kIhigh = 0;
    public static final double kDhigh = 0;
  }
  public static final class C {
    public static double Position_LOW = -70;
    public static double Position_High = 47;
    public static final double kPlow = 0.01;
    public static final double kIlow = 0;
    public static final double kDlow = 0;
    public static final double kPhigh = .02;
    public static final double kIhigh = 0;
    public static final double kDhigh = 0;
  }

  public static final int CAN_high = 21;
  public static final int CAN_low = 22;
}

public static final int taynesIntake = 7;

}