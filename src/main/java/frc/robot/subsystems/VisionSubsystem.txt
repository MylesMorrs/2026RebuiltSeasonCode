package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionSubsystem extends SubsystemBase {
  private final DriveSubsystem m_drive;
  private final PhotonCamera m_camera;
  private final PhotonPoseEstimator m_poseEstimator;
  private final AprilTagFieldLayout m_fieldLayout;

  private final VisionSystemSim m_visionSim;
  private final PhotonCameraSim m_cameraSim;

  private boolean m_warnedDisconnected = false;

  public VisionSubsystem(DriveSubsystem drive) {
    m_drive = drive;
    m_camera = new PhotonCamera(VisionConstants.kCameraName);

    m_fieldLayout = VisionConstants.kAprilTagField.loadAprilTagLayoutField();
    m_poseEstimator = new PhotonPoseEstimator(
        m_fieldLayout,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        VisionConstants.kRobotToCamera);
    m_poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    if (RobotBase.isSimulation()) {
      m_visionSim = new VisionSystemSim("main");
      m_visionSim.addAprilTags(m_fieldLayout);

      SimCameraProperties cameraProperties = new SimCameraProperties();
      cameraProperties.setCalibration(
          VisionConstants.kSimCameraWidthPx,
          VisionConstants.kSimCameraHeightPx,
          Rotation2d.fromDegrees(VisionConstants.kSimCameraDiagFovDeg));
      cameraProperties.setFPS(VisionConstants.kSimCameraFps);
      cameraProperties.setAvgLatencyMs(VisionConstants.kSimAvgLatencyMs);
      cameraProperties.setLatencyStdDevMs(VisionConstants.kSimLatencyStdDevMs);

      m_cameraSim = new PhotonCameraSim(m_camera, cameraProperties);
      m_cameraSim.enableDrawWireframe(true);
      m_visionSim.addCamera(m_cameraSim, VisionConstants.kRobotToCamera);
    } else {
      m_visionSim = null;
      m_cameraSim = null;
    }
  }

  @Override
  public void periodic() {
    int acceptedMeasurements = 0;
    int rejectedMeasurements = 0;

    var unreadResults = m_camera.getAllUnreadResults();
    SmartDashboard.putNumber("Vision/UnreadResults", unreadResults.size());

    for (PhotonPipelineResult result : unreadResults) {
      if (!result.hasTargets()) {
        continue;
      }

      Optional<EstimatedRobotPose> estimateOpt = m_poseEstimator.update(result);
      if (estimateOpt.isEmpty()) {
        continue;
      }

      EstimatedRobotPose estimate = estimateOpt.get();
      Pose2d estimatedPose = estimate.estimatedPose.toPose2d();

      if (!isPoseInsideField(estimatedPose) || Math.abs(estimate.estimatedPose.getZ()) > VisionConstants.kMaxPoseZMeters) {
        rejectedMeasurements++;
        continue;
      }

      int usedTargetCount = estimate.targetsUsed.size();
      if (usedTargetCount == 1
          && estimate.targetsUsed.get(0).getPoseAmbiguity() > VisionConstants.kMaxSingleTagAmbiguity) {
        rejectedMeasurements++;
        continue;
      }

      double nearestTagDistance = getNearestTagDistanceMeters(estimate.estimatedPose);
      if (nearestTagDistance > VisionConstants.kMaxTagDistanceMeters) {
        rejectedMeasurements++;
        continue;
      }

      Pose2d currentPose = m_drive.getPose();
      if (currentPose.getTranslation().getDistance(estimatedPose.getTranslation())
              > VisionConstants.kMaxVisionPoseDeltaMeters
          || Math.abs(
                  currentPose.getRotation().minus(estimatedPose.getRotation()).getDegrees())
              > VisionConstants.kMaxVisionPoseDeltaDeg) {
        rejectedMeasurements++;
        continue;
      }

      var visionStdDevs = computeVisionStdDevs(usedTargetCount, nearestTagDistance);
      m_drive.addVisionMeasurement(estimatedPose, estimate.timestampSeconds, visionStdDevs);
      acceptedMeasurements++;

      SmartDashboard.putNumber("Vision/TargetCount", usedTargetCount);
      SmartDashboard.putNumber("Vision/EstimatedX", estimatedPose.getX());
      SmartDashboard.putNumber("Vision/EstimatedY", estimatedPose.getY());
      SmartDashboard.putNumber("Vision/EstimatedDeg", estimatedPose.getRotation().getDegrees());
      SmartDashboard.putNumber("Vision/NearestTagDistanceM", nearestTagDistance);
    }

    SmartDashboard.putNumber("Vision/AcceptedMeasurements", acceptedMeasurements);
    SmartDashboard.putNumber("Vision/RejectedMeasurements", rejectedMeasurements);
    SmartDashboard.putBoolean("Vision/Connected", m_camera.isConnected());

    if (!m_camera.isConnected() && !m_warnedDisconnected) {
      DriverStation.reportWarning("PhotonVision camera not connected: " + VisionConstants.kCameraName, false);
      m_warnedDisconnected = true;
    } else if (m_camera.isConnected()) {
      m_warnedDisconnected = false;
    }
  }

  @Override
  public void simulationPeriodic() {
    if (m_visionSim != null) {
      m_visionSim.update(m_drive.getPose());
    }
  }

  private boolean isPoseInsideField(Pose2d pose) {
    return pose.getX() >= 0.0
        && pose.getY() >= 0.0
        && pose.getX() <= m_fieldLayout.getFieldLength()
        && pose.getY() <= m_fieldLayout.getFieldWidth();
  }

  private double getNearestTagDistanceMeters(Pose3d estimatedRobotPose) {
    Translation2d estimatedTranslation = estimatedRobotPose.toPose2d().getTranslation();
    double minDistance = Double.POSITIVE_INFINITY;

    for (var target : m_fieldLayout.getTags()) {
      Optional<Pose3d> tagPoseOpt = m_fieldLayout.getTagPose(target.ID);
      if (tagPoseOpt.isEmpty()) {
        continue;
      }
      double distance = estimatedTranslation.getDistance(tagPoseOpt.get().toPose2d().getTranslation());
      minDistance = Math.min(minDistance, distance);
    }

    return minDistance;
  }

  private Matrix<N3, N1> computeVisionStdDevs(int targetCount, double nearestTagDistanceMeters) {
    if (targetCount >= 2) {
      double distanceScale = 1.0 + (nearestTagDistanceMeters * nearestTagDistanceMeters * 0.04);
      return VecBuilder.fill(
          VisionConstants.kVisionMultiTagStdDevX * distanceScale,
          VisionConstants.kVisionMultiTagStdDevY * distanceScale,
          VisionConstants.kVisionMultiTagStdDevTheta * distanceScale);
    }

    double distanceScale = 1.0 + (nearestTagDistanceMeters * nearestTagDistanceMeters * 0.08);
    return VecBuilder.fill(
        VisionConstants.kVisionStdDevX * distanceScale,
        VisionConstants.kVisionStdDevY * distanceScale,
        VisionConstants.kVisionStdDevTheta * distanceScale);
  }
}
