package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.VisionConstants;

public class VisionSubsystem extends SubsystemBase {

  private final PhotonCamera frontCam = new PhotonCamera("frontcamera");
  private final PhotonCamera backCam  = new PhotonCamera("backcamera");

  private final AprilTagFieldLayout fieldLayout;

  private final PhotonPoseEstimator frontEstimator;
  private final PhotonPoseEstimator backEstimator;

  // Throttle console prints
  private double lastPrint = 0.0;

  public VisionSubsystem() {
    try {
      // Loads the 2026 field tag layout from WPILib's built-in resources
      fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026Rebuilt);

    } catch (Exception e) {
      throw new RuntimeException("Failed to load 2026 AprilTag field layout", e);
    }

    frontEstimator = new PhotonPoseEstimator(
        fieldLayout,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        frontCam,
        VisionConstants.ROBOT_TO_FRONT_CAM
    );

    backEstimator = new PhotonPoseEstimator(
        fieldLayout,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        backCam,
        VisionConstants.ROBOT_TO_BACK_CAM
    );

    // Fallback if multitag isn't available (single tag, still works)
    frontEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    backEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  }

  /** Get pose estimate from front cam (if any) */
  public Optional<EstimatedRobotPose> getFrontEstimatedPose() {
    return frontEstimator.update();
  }

  /** Get pose estimate from back cam (if any) */
  public Optional<EstimatedRobotPose> getBackEstimatedPose() {
    return backEstimator.update();
  }

  @Override
  public void periodic() {
    var front = getFrontEstimatedPose();
    var back  = getBackEstimatedPose();

    // Publish to SmartDashboard for tomorrow
    publish("Front", front);
    publish("Back", back);

    // Also print occasionally so you can see it in logs without a dashboard
    double now = Timer.getFPGATimestamp();
    if (now - lastPrint > 0.5) { // twice per second
      lastPrint = now;
      if (front.isPresent()) {
        Pose2d p = front.get().estimatedPose.toPose2d();
        System.out.printf("[Vision] Front pose: x=%.2f y=%.2f deg=%.1f%n",
            p.getX(), p.getY(), p.getRotation().getDegrees());
      }
      if (back.isPresent()) {
        Pose2d p = back.get().estimatedPose.toPose2d();
        System.out.printf("[Vision] Back  pose: x=%.2f y=%.2f deg=%.1f%n",
            p.getX(), p.getY(), p.getRotation().getDegrees());
      }
    }
  }

  private void publish(String name, Optional<EstimatedRobotPose> est) {
    String base = "Vision/" + name + "/";
    if (est.isEmpty()) {
      SmartDashboard.putBoolean(base + "HasPose", false);
      return;
    }

    Pose2d pose = est.get().estimatedPose.toPose2d();
    SmartDashboard.putBoolean(base + "HasPose", true);
    SmartDashboard.putNumber(base + "X_m", pose.getX());
    SmartDashboard.putNumber(base + "Y_m", pose.getY());
    SmartDashboard.putNumber(base + "Rot_deg", pose.getRotation().getDegrees());
    SmartDashboard.putNumber(base + "Timestamp_s", est.get().timestampSeconds);
  }
}
