package frc.robot.util;

import org.photonvision.PhotonCamera;

public class VisionCamera {
    private PhotonCamera camera;
    private boolean enabled;
    private boolean singleTagEstimation = false;

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

        public VisionCamera build() {
            return new VisionCamera(this);
        }
    }
}
