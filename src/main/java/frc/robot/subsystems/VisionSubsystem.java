package frc.robot.subsystems;

import java.util.HashSet;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.Set;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase {
  private final PhotonCamera camera;
  private PhotonPipelineResult latestResult;

  public VisionSubsystem() {
    camera = new PhotonCamera(VisionConstants.kCameraName);
    latestResult = null;
  }

  private Optional<PhotonTrackedTarget> getBestAimTarget() {
    if (latestResult == null || !latestResult.hasTargets()) {
      SmartDashboard.putBoolean("Vision/UsingAllowedTag", false);
      SmartDashboard.putNumber("Vision/TrackedTagId", -1);
      return Optional.empty();
    }

    var allianceOpt = DriverStation.getAlliance();
    Set<Integer> allowedIds;
    if (allianceOpt.isPresent()) {
      allowedIds =
          allianceOpt.get() == DriverStation.Alliance.Red
              ? VisionConstants.kRedAllianceAimTagIds
              : VisionConstants.kBlueAllianceAimTagIds;
    } else {
      // Fallback for practice/testing when alliance is unknown in DS.
      allowedIds = new HashSet<>(VisionConstants.kBlueAllianceAimTagIds);
      allowedIds.addAll(VisionConstants.kRedAllianceAimTagIds);
    }

    PhotonTrackedTarget bestAllowed = null;
    for (PhotonTrackedTarget target : latestResult.getTargets()) {
      int id = target.getFiducialId();
      if (allowedIds.contains(id) && (bestAllowed == null || target.getArea() > bestAllowed.getArea())) {
        bestAllowed = target;
      }
    }

    if (bestAllowed == null) {
      if (!VisionConstants.kAllowAnyTagWhenNoAllowedSeen) {
        SmartDashboard.putBoolean("Vision/UsingAllowedTag", false);
        SmartDashboard.putNumber("Vision/TrackedTagId", -1);
        return Optional.empty();
      }

      PhotonTrackedTarget bestAny = latestResult.getBestTarget();
      SmartDashboard.putBoolean("Vision/UsingAllowedTag", false);
      SmartDashboard.putNumber("Vision/TrackedTagId", bestAny.getFiducialId());
      return Optional.of(bestAny);
    }

    SmartDashboard.putBoolean("Vision/UsingAllowedTag", true);
    SmartDashboard.putNumber("Vision/TrackedTagId", bestAllowed.getFiducialId());
    return Optional.of(bestAllowed);
  }

  public OptionalDouble getBestTargetYawDegrees() {
    var targetOpt = getBestAimTarget();
    if (targetOpt.isEmpty()) {
      return OptionalDouble.empty();
    }
    return OptionalDouble.of(targetOpt.get().getYaw());
  }

  public OptionalDouble getBestTargetDistanceMeters() {
    var targetOpt = getBestAimTarget();
    if (targetOpt.isEmpty()) {
      return OptionalDouble.empty();
    }
    return OptionalDouble.of(targetOpt.get().getBestCameraToTarget().getTranslation().getNorm());
  }

  @Override
  public void periodic() {
    var unread = camera.getAllUnreadResults();
    if (!unread.isEmpty()) {
      latestResult = unread.get(unread.size() - 1);
    }

    SmartDashboard.putBoolean("Vision/Connected", camera.isConnected());
    SmartDashboard.putBoolean("Vision/HasTarget", latestResult != null && latestResult.hasTargets());
    SmartDashboard.putBoolean("Vision/AllianceKnown", DriverStation.getAlliance().isPresent());
    SmartDashboard.putNumber("Vision/TargetCount", latestResult == null ? 0 : latestResult.getTargets().size());
  }
}
