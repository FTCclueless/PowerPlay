package org.firstinspires.ftc.teamcode.vision;

public class TfodCameraControlData {
    public int tfodCameraGain;
    public int tfodCameraExposure;
    public boolean tfodAutoExposurePriority;
    public double tfodZoom;
    public double tfodAspectRatio;
    public TfodCameraControlData(int cameraGain, int cameraExposure, boolean ae, double zoom, double aspectRatio) {
        tfodCameraExposure = cameraExposure;
        tfodCameraGain = cameraGain;
        tfodAutoExposurePriority = ae;
        tfodZoom = zoom;
        tfodAspectRatio = aspectRatio;
    }
    public boolean equal(TfodCameraControlData a) {
        return ((tfodAutoExposurePriority == a.tfodAutoExposurePriority) &&
                (tfodZoom == a.tfodZoom) && (tfodAutoExposurePriority == a.tfodAutoExposurePriority) &&
                (tfodCameraGain == a.tfodCameraGain) && (tfodAspectRatio == a.tfodAspectRatio));
    }
    public String toString()
    {
        return "tfodCameraExposure " + tfodCameraExposure + " tfodCameraGain " + tfodCameraGain +
                " tfodAutoExposurePriority " + tfodAutoExposurePriority +
                " tfodAspectRatio " + tfodAspectRatio + " tfodZoom " + tfodZoom;

    }
    public void clone(TfodCameraControlData a) {
        tfodCameraExposure = a.tfodCameraExposure;
        tfodCameraGain = a.tfodCameraGain;
        tfodAutoExposurePriority = a.tfodAutoExposurePriority;
        tfodZoom = a.tfodZoom;
        tfodAspectRatio = a.tfodAspectRatio;
    }
}
