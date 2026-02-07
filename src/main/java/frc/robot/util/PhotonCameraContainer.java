package frc.robot.util;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PoseConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class PhotonCameraContainer {
    private static ArrayList<VisionCamera> cameras = new ArrayList<VisionCamera>();
    public static int cameraCount = 0;

    public static void addPhotonCamera(String name) {
        cameras.add(new VisionCamera(name));
        cameraCount++;
    }

    public static void addPhotonCamera(VisionCamera camera) {
        cameras.add(camera);
        cameraCount++;
    }

    public static void estimateVisionOdometry(CommandSwerveDrivetrain odometry) {

        for (VisionCamera visionCamera : cameras) {
            if (visionCamera.isEnabled()) {
                PhotonCamera camera = visionCamera.getPhotonCamera();
                List<PhotonPipelineResult> results = camera.getAllUnreadResults();
                visionCamera.isSingleTagEstimationEnabled();

                for (PhotonPipelineResult result : results) {
                    if (result.getMultiTagResult().isPresent()) {
                        Transform3d multiTagPose = result.getMultiTagResult().get().estimatedPose.best;
                        Pose2d pose = toPose2D(multiTagPose);
                        odometry.addVisionMeasurement(pose, result.getTimestampSeconds());

                    } 
                    // else if (visionCamera.isSingleTagEstimationEnabled() && result.getBestTarget() != null) {

                    //     Transform3d tagToCamera = result.getBestTarget().altCameraToTarget;
                    //     Pose3d tagToField = PoseConstants.kAprilTagFieldLayout
                    //             .getTagPose(result.getBestTarget().fiducialId).get();

                    //     if (tagToField == null)
                    //         continue;

                    //     Pose3d robotPose = tagToField.transformBy(tagToCamera);//.rotateBy(new Rotation3d(0,0,DriveConstants.GYRO_ANGLE_OFFSET.getRadians()));
                    //     // robotPose = robotPose.rotateAround(robotPose.getTranslation(), new Rotation3d(0,0,DriveConstants.GYRO_ANGLE_OFFSET.getRadians()));
                    //     odometry.addVisionMeasurement(robotPose.toPose2d(), result.getTimestampSeconds());
                    // }
                }

            }

        }

    }

    public static Pose2d toPose2D(Transform3d pose) {
        return new Pose2d(pose.getTranslation().toTranslation2d(), pose.getRotation().toRotation2d());
    }
}
