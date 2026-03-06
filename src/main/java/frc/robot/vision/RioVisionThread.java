// ============================================================================
// FILE: src/main/java/frc/robot/vision/RioVisionThread.java
//
// PURPOSE: Background daemon thread that captures USB camera frames and runs
//   WPILib AprilTag detection directly on the roboRIO 2.  Publishes the best
//   alliance HUB tag as a VisionResult via AtomicReference (zero main-loop
//   cost).
//
// DESIGN:
//   - 320x240 grayscale at ~15 fps (CvSink blocks until next frame)
//   - Uses WPILib AprilTagDetector (36h11 family) — detect-only, no solvePnP
//   - Filters for alliance HUB tags and picks the largest (closest)
//   - Computes yaw/pitch from pixel coordinates using camera FOV
//   - Reuses a single Mat per iteration to avoid GC pressure
//   - Daemon thread: JVM will not wait for it on shutdown
//
// HARDWARE: Logitech C920 (HD Pro) USB camera
//   - Horizontal FOV: ~70.42 deg
//   - At 320x240 (4:3 crop): Vertical FOV: ~43.3 deg
// ============================================================================
package frc.robot.vision;

import java.util.concurrent.atomic.AtomicReference;

import org.opencv.core.Mat;

import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.Constants;

public class RioVisionThread extends Thread {

    private final AtomicReference<VisionResult> latestResult;
    private final AtomicReference<Double> lastFrameTimestampSec;

    public RioVisionThread(
            AtomicReference<VisionResult> latestResult,
            AtomicReference<Double> lastFrameTimestampSec) {
        super("RioVisionThread");
        setDaemon(true);
        this.latestResult = latestResult;
        this.lastFrameTimestampSec = lastFrameTimestampSec;
    }

    @Override
    public void run() {
        // Start USB camera via CameraServer (also streams to dashboard)
        UsbCamera usbCamera = CameraServer.startAutomaticCapture(
                Constants.Vision.CAMERA_DEVICE_ID);
        usbCamera.setResolution(Constants.Vision.CAMERA_WIDTH, Constants.Vision.CAMERA_HEIGHT);
        usbCamera.setFPS(Constants.Vision.CAMERA_FPS);

        CvSink cvSink = CameraServer.getVideo();

        // Configure AprilTag detector for 36h11 tag family
        AprilTagDetector detector = new AprilTagDetector();
        detector.addFamily("tag36h11");

        // Reuse Mats to avoid GC pressure in the hot loop
        Mat mat = new Mat();
        Mat grayMat = new Mat();

        while (!Thread.interrupted()) {
            // grabFrame blocks until the next frame arrives (or timeout)
            long frameTime = cvSink.grabFrame(mat);
            if (frameTime == 0) {
                // Frame grab failed — camera probably disconnected.
                // Retry on next iteration; main loop sees stale result.
                continue;
            }

            double timestampSec = Timer.getFPGATimestamp();
            lastFrameTimestampSec.set(timestampSec);

            Mat detectorFrame = VisionSupport.prepareDetectorFrame(mat, grayMat);
            AprilTagDetection[] detections = detector.detect(detectorFrame);
            if (detections.length == 0) {
                continue;
            }

            // Filter for alliance HUB tags and pick the largest (closest)
            int[] hubTagIds = getAllianceHubTagIds();
            AprilTagDetection best = null;
            double bestPixelHeight = 0;

            for (AprilTagDetection det : detections) {
                if (!isHubTag(det.getId(), hubTagIds)) {
                    continue;
                }
                double pixelHeight = tagPixelHeight(det);
                if (pixelHeight > bestPixelHeight) {
                    best = det;
                    bestPixelHeight = pixelHeight;
                }
            }

            if (best == null) {
                continue;
            }

            // Compute tag center from corner coordinates
            double centerX = (best.getCornerX(0) + best.getCornerX(1)
                    + best.getCornerX(2) + best.getCornerX(3)) / 4.0;
            double centerY = (best.getCornerY(0) + best.getCornerY(1)
                    + best.getCornerY(2) + best.getCornerY(3)) / 4.0;

            double imageCenterX = Constants.Vision.CAMERA_WIDTH / 2.0;
            double imageCenterY = Constants.Vision.CAMERA_HEIGHT / 2.0;

            // Pixel-to-angle math (from the fallback plan doc)
            double yawDeg = (centerX - imageCenterX) / imageCenterX
                    * (Constants.Vision.HORIZONTAL_FOV_DEG / 2.0);
            double pitchDeg = (imageCenterY - centerY) / imageCenterY
                    * (Constants.Vision.VERTICAL_FOV_DEG / 2.0);

            latestResult.set(new VisionResult(
                    best.getId(), yawDeg, pitchDeg, bestPixelHeight, timestampSec));
        }

        detector.close();
        grayMat.release();
        mat.release();
    }

    private static double tagPixelHeight(AprilTagDetection det) {
        // Vertical span of the tag corners
        double minY = Math.min(Math.min(det.getCornerY(0), det.getCornerY(1)),
                Math.min(det.getCornerY(2), det.getCornerY(3)));
        double maxY = Math.max(Math.max(det.getCornerY(0), det.getCornerY(1)),
                Math.max(det.getCornerY(2), det.getCornerY(3)));
        return maxY - minY;
    }

    private static int[] getAllianceHubTagIds() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red
                    ? Constants.Vision.RED_HUB_TAG_IDS
                    : Constants.Vision.BLUE_HUB_TAG_IDS;
        }
        // Alliance unknown — accept either HUB
        return null;
    }

    private static boolean isHubTag(int fiducialId, int[] hubTagIds) {
        if (hubTagIds == null) {
            // Accept either alliance's tags
            for (int id : Constants.Vision.RED_HUB_TAG_IDS) {
                if (id == fiducialId) return true;
            }
            for (int id : Constants.Vision.BLUE_HUB_TAG_IDS) {
                if (id == fiducialId) return true;
            }
            return false;
        }
        for (int id : hubTagIds) {
            if (id == fiducialId) return true;
        }
        return false;
    }
}
