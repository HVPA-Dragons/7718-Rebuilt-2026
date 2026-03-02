package frc.robot;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;

public final class VisionConstants {
  private VisionConstants() {}

  // TODO: replace with YOUR measured values (meters)
  public static final Transform3d ROBOT_TO_FRONT_CAM =
      new Transform3d(
          new Translation3d(0.25, 0.10, 0.55), // x, y, z
          new Rotation3d(0.0, 0.0, 0.0)        // roll, pitch, yaw (radians)
      );

  public static final Transform3d ROBOT_TO_BACK_CAM =
      new Transform3d(
          new Translation3d(-0.20, -0.10, 0.55),
          new Rotation3d(0.0, 0.0, Math.PI)    // 180° yaw
      );
}
