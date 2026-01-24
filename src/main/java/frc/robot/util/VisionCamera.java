package frc.robot.util;

import org.photonvision.PhotonCamera;

public class VisionCamera {
    private PhotonCamera camera;
    private boolean enabled;

    public VisionCamera(String name, boolean enabled) {
        this.camera = new PhotonCamera(name);
        this.enabled = enabled;
    }

    public VisionCamera(String name) {
        this.camera = new PhotonCamera(name);
        this.enabled = true;
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


}
