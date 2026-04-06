package frc.robot.util;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Transform3d;

public class VisionCamera {
    private PhotonCamera camera;
    private boolean enabled;
    private boolean singleTagEstimation = false;
    private Transform3d offset = new Transform3d();

    public VisionCamera(String name, boolean enabled) {
        this.camera = new PhotonCamera(name);
        this.enabled = enabled;
    }

    public VisionCamera(String name) {
        this.camera = new PhotonCamera(name);
        this.enabled = true;
    }

    private VisionCamera(Builder builder) {
        this.camera = new PhotonCamera(builder.name);
        this.enabled = builder.isEnabled;
        this.singleTagEstimation = builder.isSingleTagEstimationEnabled;
        this.offset = builder.offset;
    }

    public Transform3d getCameraOffset() {
        return offset;
    }

    public boolean isSingleTagEstimationEnabled() {
        return singleTagEstimation;
    }

    public boolean isEnabled() {
        return enabled;
    }

    public PhotonCamera getPhotonCamera() {
        return camera;
    }

    public void enable() {
        enabled = true;
    }

    public void disable() {
        enabled = false;
    }

    public static class Builder {
        private String name;
        private boolean isEnabled = true;
        private boolean isSingleTagEstimationEnabled = false;
        private Transform3d offset = new Transform3d();

        public Builder withName(String name) {
            this.name = name;
            return this;
        }

        public Builder withSingleTagEstimation() {
            this.isSingleTagEstimationEnabled = true;
            return this;
        }

        public Builder withCameraEnabled() {
            this.isEnabled = true;
            return this;
        }

        public Builder withCameraDisabled() {
            this.isEnabled = false;
            return this;
        }

        public Builder withOffset(Transform3d offTransform3d) {
            this.offset = offTransform3d;
            return this;
        }

        public VisionCamera build() {
            return new VisionCamera(this);
        }
    }
}
