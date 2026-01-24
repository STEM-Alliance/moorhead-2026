package frc.robot.util;

import java.util.ArrayList;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants.PoseConstants;

public class PhotonCameraContainer {
    private static ArrayList<VisionCamera> cameras = new ArrayList<VisionCamera>();
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

                for (PhotonPipelineResult result : results) {
                    if (result.getMultiTagResult().isPresent()) {
                        Transform3d multiTagPose = result.getMultiTagResult().get().estimatedPose.best;
                        odometry.addVisionMeasurement(toPose2D(multiTagPose), result.getTimestampSeconds());
                        

                    } else if (result.getBestTarget() != null) {
                        var tagToCamera = result.getBestTarget().altCameraToTarget;

                        Pose3d tagToField = PoseConstants.kAprilTagFieldLayout
                                .getTagPose(result.getBestTarget().fiducialId).get();

                        if (tagToField == null)
                            continue;

                        Pose3d robotPose = tagToField.transformBy(tagToCamera);

                        odometry.addVisionMeasurement(robotPose.toPose2d(), result.getTimestampSeconds());
                    }
                }

            }

        }

    }

    public static Pose2d toPose2D(Transform3d pose) {
        return new Pose2d(pose.getTranslation().toTranslation2d(), pose.getRotation().toRotation2d());
    }
}
