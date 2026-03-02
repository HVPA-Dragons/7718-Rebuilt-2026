package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionSubsystem extends SubsystemBase {

    private PhotonCamera frontCamera = new PhotonCamera("frontcamera");
    private PhotonCamera backCamera = new PhotonCamera("backcamera");

    public VisionSubsystem() {}

    @Override
    public void periodic() {

        var frontResult = frontCamera.getLatestResult();
        var backResult = backCamera.getLatestResult();

        if (frontResult.hasTargets()) {
            PhotonTrackedTarget target = frontResult.getBestTarget();
            System.out.println("Front Cam Tag ID: " + target.getFiducialId());
            System.out.println("Front Cam Yaw: " + target.getYaw());
            System.out.println("Front Cam Pitch: " + target.getPitch());
        }

        if (backResult.hasTargets()) {
            PhotonTrackedTarget target = backResult.getBestTarget();
            System.out.println("Back Cam Tag ID: " + target.getFiducialId());
            System.out.println("Back Cam Yaw: " + target.getYaw());
            System.out.println("Back Cam Pitch: " + target.getPitch());
        }
    }
}
