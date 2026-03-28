// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import java.util.Set;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 12;
    public static final double kMaxAngularSpeed = 4 * Math.PI; // radians per second

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(26.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(26.5);
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
    public static final int kFrontLeftDrivingCanId = 1;
    public static final int kRearLeftDrivingCanId = 15;
    public static final int kFrontRightDrivingCanId = 3;
    public static final int kRearRightDrivingCanId = 8;

    public static final int kFrontLeftTurningCanId = 2;
    public static final int kRearLeftTurningCanId = 14;
    public static final int kFrontRightTurningCanId = 4;
    public static final int kRearRightTurningCanId = 16;

    public static final boolean kGyroReversed = false;

    public static final Matrix<N3, N1> kDefaultVisionMeasurementStdDevs = VecBuilder.fill(
        VisionConstants.kVisionStdDevX,
        VisionConstants.kVisionStdDevY,
        VisionConstants.kVisionStdDevTheta);
  }

  public static final class IntakeConstants
  {
      
      public static final int FuelIntakeCanId = 5; 
      public static final int IntakePivotCanId = 6;
       

      public static final double kAutoIntakeSpeed = -0.8;
      public static final double kAutoIntakeSeconds = 1.0;
      public static final double kAutoIntakeDropSpeed = 0.6;
      public static final double kAutoIntakeDropSeconds = 0.5;

 



  }


  public static final class ShootingConstants
  {
      
      public static final int ShooterMotorCanId = 12; 
      //TODO set can id for spindexter
      public static final int SpindexerMotorCanID = 14; 
      public static final int TransferMotorCanId = 13;
      public static final int AimingMotorCanId = 11; 
      public static final double kAutoTransferSpeed = 0.8;
      public static final double kAutoShootSeconds = 1.0;
      public static final double kAutoShootMinPower = 0.45;
      public static final double kAutoShootMaxPower = 0.95;
      public static final double kAutoShootMinDistanceMeters = 1.0;
      public static final double kAutoShootMaxDistanceMeters = 4.0;
      public static final double kAutoShootPowerOffsetStepPerLoop = 0.01;
      public static final double kAutoShootPowerOffsetMax = 0.25;
      public static final int kTurretEncoderDioChannel = 0; // TODO: set DIO channel for REV Through Bore
      public static final double kTurretEncoderGearRatio = 1.0; // TODO: set actual turret gear ratio
      public static final double kTurretEncoderOffsetDeg = 0.0; // TODO: set mechanical zero offset
      public static final double kTurretSearchMaxDegrees = 90.0;
      public static final double kTurretSearchSpeed = 0.2;
      public static final double TurretAimKp = 0.02;
      public static final double TurretAimMaxSpeed = 0.35;
      public static final double TurretAimDeadbandDeg = 1.0;
      // Positive value means hold the tag this many degrees off center.
      public static final double TurretAimYawOffsetDeg = 0.0;
      // Hold right bumper while auto-aiming to run shooter at fixed power.
      public static final double TurretAutoShootPower = 0.35;
      // Auto-shoot only when yaw error is within this tolerance.
      public static final double TurretShootMaxYawErrorDeg = 2.5;

  }

public static final class ClimbingConstants
{
   
    public static final int ClimbingMotor1CanId = 10; 
    //TODO set can id for climbing motor2
    public static final int ClimbingMotor2CanId = 5; 

    

}


  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kManipulatorControllerPort = 1; 
    public static final double kDriveDeadband = 0.05;
    // Teleop input gain to make drivetrain feel more responsive.
    public static final double kDriveInputGain = 2.3;
    public static final double kTurnInputGain = 2.3;
    public static final double kDriveSlowMultiplier = 0.4;

  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
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

  public static final class VisionConstants {
    // Camera name must match PhotonVision UI exactly.
    public static final String kCameraName = "Depstech_webcam";

    // Robot-to-camera transform (meters/radians): +X forward, +Y left, +Z up.
    public static final Transform3d kRobotToCamera = new Transform3d(
        new Translation3d(0.28, 0.0, 0.23),
        new Rotation3d(0.0, 0.0, 0.0));

    // Keep this at kDefaultField until you lock in event field type.
    public static final AprilTagFields kAprilTagField = AprilTagFields.kDefaultField;

    // Turret auto-aim target filtering by alliance.
    // Update these IDs to match your strategy for the current game.
    public static final Set<Integer> kBlueAllianceAimTagIds = Set.of(21, 24, 25,26, 27, 18);
    public static final Set<Integer> kRedAllianceAimTagIds = Set.of(1);
    // If true, use best visible tag when none match alliance filter.
    public static final boolean kAllowAnyTagWhenNoAllowedSeen = true;

    // Pose estimator model noise (odometry process noise): x, y, theta.
    public static final double kStateStdDevX = 0.05;
    public static final double kStateStdDevY = 0.05;
    public static final double kStateStdDevTheta = 0.08;

    // Base vision noise for single-tag updates: x, y, theta.
    public static final double kVisionStdDevX = 1.0;
    public static final double kVisionStdDevY = 1.0;
    public static final double kVisionStdDevTheta = 1.8;

    // Better trust for multi-tag solutions.
    public static final double kVisionMultiTagStdDevX = 0.25;
    public static final double kVisionMultiTagStdDevY = 0.25;
    public static final double kVisionMultiTagStdDevTheta = 0.6;

    // Measurement gating.
    public static final double kMaxSingleTagAmbiguity = 0.20;
    public static final double kMaxTagDistanceMeters = 4.5;
    public static final double kMaxPoseZMeters = 0.35;
    public static final double kMaxVisionPoseDeltaMeters = 2.0;
    public static final double kMaxVisionPoseDeltaDeg = 35.0;

    // Simulation camera properties.
    public static final int kSimCameraWidthPx = 1280;
    public static final int kSimCameraHeightPx = 720;
    public static final double kSimCameraDiagFovDeg = 90.0;
    public static final double kSimCameraFps = 30.0;
    public static final double kSimAvgLatencyMs = 35.0;
    public static final double kSimLatencyStdDevMs = 8.0;

  }
}
