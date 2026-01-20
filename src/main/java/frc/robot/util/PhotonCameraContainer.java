package frc.robot.util;

import java.util.ArrayList;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;

public class PhotonCameraContainer {
    private static ArrayList<VisionCamera> cameras;
    public static int cameraCount = 0;

    public static void addPhotonCamera(String name) {
        cameras.add(new VisionCamera(name));
        cameraCount++;
    }

    public static void estimateVisionOdometry(SwerveDrivePoseEstimator odometry) {
        for (VisionCamera visionCamera : cameras) {
            if (visionCamera.isEnabled()) {
                PhotonCamera camera = visionCamera.getPhotonCamera();
                var results = camera.getAllUnreadResults();
                for (var result : results) {
                    var multiTagResult = result.getMultiTagResult();
                    if (multiTagResult.isPresent()) {
                        var fieldToCamera = multiTagResult.get().estimatedPose.best;
                        odometry.addVisionMeasurement(toPose2D(fieldToCamera), result.getTimestampSeconds());
                    }
                }
            }

        }

    }

    private static Pose2d toPose2D(Transform3d pose) {
        return new Pose2d(pose.getTranslation().toTranslation2d(), pose.getRotation().toRotation2d());
    }
}
