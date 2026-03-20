package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import java.io.File;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class DriveSubsystem extends SubsystemBase {
  private final SwerveDrive swerveDrive;

  public DriveSubsystem() {
    try {
      SwerveDriveTelemetry.verbosity = TelemetryVerbosity.LOW;
      File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
      swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(DriveConstants.kMaxSpeedMetersPerSecond);
      swerveDrive.useExternalFeedbackSensor();
      swerveDrive.setHeadingCorrection(false, 0.0);
      swerveDrive.setCosineCompensator(false);
      swerveDrive.setAngularVelocityCompensation(true, true, 0.1);
      swerveDrive.setModuleEncoderAutoSynchronize(true, 1.0);
      swerveDrive.setVisionMeasurementStdDevs(
          DriveConstants.kDefaultVisionMeasurementStdDevs);
    } catch (Exception e) {
      throw new RuntimeException("Failed to initialize YAGSL from src/main/deploy/swerve", e);
    }

    RobotConfig config = null;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      e.printStackTrace();
    }

    if (config == null) {
      DriverStation.reportError(
          "PathPlanner settings.json missing! Open PathPlanner GUI -> Robot Settings -> Save to src/main/deploy/pathplanner/",
          false);
      return;
    }

    AutoBuilder.configure(
        this::getPose,
        this::resetOdometry,
        this::getRobotRelativeSpeeds,
        (speeds, ffs) -> driveRobotRelative(speeds),
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0),
            new PIDConstants(5.0, 0.0, 0.0)),
        config,
        () -> DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == DriverStation.Alliance.Red,
        this);
  }

  private ChassisSpeeds getRobotRelativeSpeeds() {
    return swerveDrive.getRobotVelocity();
  }

  private void driveRobotRelative(ChassisSpeeds speeds) {
    swerveDrive.setChassisSpeeds(speeds);
  }

  private SwerveModuleState[] getModuleStates() {
    return swerveDrive.getStates();
  }

  private SwerveModulePosition[] getModulePositions() {
    return swerveDrive.getModulePositions();
  }

  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  public Field2d getField() {
    return swerveDrive.field;
  }

  public void resetOdometry(Pose2d pose) {
    swerveDrive.resetOdometry(pose);
  }

  public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
    swerveDrive.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds);
  }

  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    swerveDrive.addVisionMeasurement(
        visionRobotPoseMeters,
        timestampSeconds,
        visionMeasurementStdDevs);
  }

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

    ChassisSpeeds requestedSpeeds = new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered);
    if (fieldRelative) {
      swerveDrive.driveFieldOriented(
          ChassisSpeeds.fromFieldRelativeSpeeds(
              requestedSpeeds.vxMetersPerSecond,
              requestedSpeeds.vyMetersPerSecond,
              requestedSpeeds.omegaRadiansPerSecond,
              swerveDrive.getOdometryHeading()));
      return;
    }

    swerveDrive.drive(requestedSpeeds, new Translation2d());
  }

  public void setX() {
    swerveDrive.lockPose();
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    swerveDrive.setModuleStates(desiredStates, false);
  }

  public void resetEncoders() {
    swerveDrive.resetDriveEncoders();
  }

  public void zeroHeading() {
    swerveDrive.zeroGyro();
  }

  public double getHeading() {
    return swerveDrive.getOdometryHeading().getDegrees();
  }

  public double getTurnRate() {
    return getRobotRelativeSpeeds().omegaRadiansPerSecond;
  }
}
