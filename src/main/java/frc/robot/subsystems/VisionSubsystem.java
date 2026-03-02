package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.VisionConstants;

public class VisionSubsystem extends SubsystemBase {

  private final PhotonCamera frontCamera = new PhotonCamera("frontcamera");
  private final PhotonCamera backCamera  = new PhotonCamera("backcamera");

  private final AprilTagFieldLayout fieldLayout;

  private final PhotonPoseEstimator frontEstimator;
  private final PhotonPoseEstimator backEstimator;

  // Throttle logs so we don’t get spammed
  private double lastPrint = 0.0;

  public VisionSubsystem() {
    try {
      fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026Rebuilt);
    } catch (Exception e) {
      throw new RuntimeException("Failed to load 2026 AprilTag field layout", e);
    }

    // Pose estimator for front camera
    frontEstimator = new PhotonPoseEstimator(
        fieldLayout,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        frontCamera,
        VisionConstants.ROBOT_TO_FRONT_CAM
    );
    frontEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    // Pose estimator for back camera
    backEstimator = new PhotonPoseEstimator(
        fieldLayout,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        backCamera,
        VisionConstants.ROBOT_TO_BACK_CAM
    );
    backEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  }

  // Pose estimate from front camera, if available 
  public Optional<EstimatedRobotPose> getFrontEstimatedPose() {
    return frontEstimator.update();
  }

  // Pose estimate from back camera, if available 
  public Optional<EstimatedRobotPose> getBackEstimatedPose() {
    return backEstimator.update();
  }

  @Override
  public void periodic() {
    // raw detections
    var frontResult = frontCamera.getLatestResult();
    var backResult  = backCamera.getLatestResult();

    // pose estimations
    var frontPose = getFrontEstimatedPose();
    var backPose  = getBackEstimatedPose();

    // Publish pose to SmartDashboard
    publishPose("Front", frontPose);
    publishPose("Back", backPose);

    // Throttled console print (works even without dashboard)
    double now = Timer.getFPGATimestamp();
    if (now - lastPrint > 0.5) {
      lastPrint = now;

      if (frontResult.hasTargets()) {
        PhotonTrackedTarget t = frontResult.getBestTarget();
        System.out.printf("[Vision] Front best tag=%d yaw=%.1f pitch=%.1f%n",
            t.getFiducialId(), t.getYaw(), t.getPitch());
      } else {
        System.out.println("[Vision] Front: no targets");
      }

      if (backResult.hasTargets()) {
        PhotonTrackedTarget t = backResult.getBestTarget();
        System.out.printf("[Vision] Back  best tag=%d yaw=%.1f pitch=%.1f%n",
            t.getFiducialId(), t.getYaw(), t.getPitch());
      } else {
        System.out.println("[Vision] Back: no targets");
      }

      if (frontPose.isPresent()) {
        Pose2d p = frontPose.get().estimatedPose.toPose2d();
        System.out.printf("[Vision] Front pose: x=%.2f y=%.2f deg=%.1f used=%d%n",
            p.getX(), p.getY(), p.getRotation().getDegrees(), frontPose.get().targetsUsed.size());
      } else {
        System.out.println("[Vision] Front pose: none");
      }

      if (backPose.isPresent()) {
        Pose2d p = backPose.get().estimatedPose.toPose2d();
        System.out.printf("[Vision] Back  pose: x=%.2f y=%.2f deg=%.1f used=%d%n",
            p.getX(), p.getY(), p.getRotation().getDegrees(), backPose.get().targetsUsed.size());
      } else {
        System.out.println("[Vision] Back pose: none");
      }
    }
  }

  private void publishPose(String name, Optional<EstimatedRobotPose> est) {
    String base = "Vision/" + name + "/";

    SmartDashboard.putBoolean(base + "HasPose", est.isPresent());

    if (est.isEmpty()) return;

    Pose2d pose = est.get().estimatedPose.toPose2d();
    SmartDashboard.putNumber(base + "X_m", pose.getX());
    SmartDashboard.putNumber(base + "Y_m", pose.getY());
    SmartDashboard.putNumber(base + "Rot_deg", pose.getRotation().getDegrees());
    SmartDashboard.putNumber(base + "Timestamp_s", est.get().timestampSeconds);
    SmartDashboard.putNumber(base + "TagCount", est.get().targetsUsed.size());
  }
}
