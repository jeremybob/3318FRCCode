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

import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CameraServerJNI;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.UsbCameraInfo;
import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.Constants;

public class RioVisionThread extends Thread {

    private final UsbCamera sharedCamera;
    private final AtomicReference<VisionResult> latestResult;
    private final AtomicReference<Double> lastFrameTimestampSec;
    private final AtomicReference<CameraDebugInfo> cameraDebugInfo;

    public RioVisionThread(
            UsbCamera sharedCamera,
            AtomicReference<VisionResult> latestResult,
            AtomicReference<Double> lastFrameTimestampSec,
            AtomicReference<CameraDebugInfo> cameraDebugInfo) {
        super("RioVisionThread");
        setDaemon(true);
        this.sharedCamera = sharedCamera;
        this.latestResult = latestResult;
        this.lastFrameTimestampSec = lastFrameTimestampSec;
        this.cameraDebugInfo = cameraDebugInfo;
    }

    @Override
    public void run() {
        UsbCamera usbCamera = null;
        CvSink cvSink = null;
        CvSource overlayOutput = null;
        MjpegServer overlayServer = null;
        AprilTagDetector detector = null;
        Mat mat = new Mat();
        Mat grayMat = new Mat();
        double lastGrabErrorLogSec = Double.NEGATIVE_INFINITY;
        boolean firstFrameLogged = false;
        long frameCount = 0;
        int consecutiveGrabFailures = 0;
        boolean disconnectLogged = false;

        try {
            UsbCameraInfo[] cameras = CameraServerJNI.enumerateUsbCameras();
            String enumeratedSummary = summarizeUsbCameras(cameras);
            UsbCameraInfo configuredCamera = findCameraInfo(cameras, Constants.Vision.CAMERA_DEVICE_ID);
            cameraDebugInfo.set(CameraDebugInfo.defaultState()
                    .withEnumeration(enumeratedSummary)
                    .withActiveCamera(
                            configuredCamera != null ? configuredCamera.dev : -1,
                            configuredCamera != null ? configuredCamera.name : "",
                            configuredCamera != null ? configuredCamera.path : "")
                    .withStatus(cameras.length == 0 ? "NO_USB_CAMERAS" : "USB_ENUMERATED"));
            logEnumeratedUsbCameras(cameras);

            usbCamera = sharedCamera;
            if (usbCamera == null) {
                throw new IllegalStateException("Shared USB camera was not created");
            }
            UsbCameraInfo activeCamera = safeGetCameraInfo(usbCamera);
            CameraDebugInfo attachedState = cameraDebugInfo.get().withStatus("CAPTURE_ATTACHED");
            if (activeCamera != null) {
                attachedState = attachedState.withActiveCamera(
                        activeCamera.dev,
                        activeCamera.name,
                        activeCamera.path);
            }
            cameraDebugInfo.set(attachedState);

            // Use the CameraServer-managed sink for the shared automatic-capture
            // camera. A manually constructed CvSink stopped receiving frames on
            // the roboRIO even while the raw MJPEG stream remained available.
            cvSink = CameraServer.getVideo(usbCamera);
            cvSink.setEnabled(true);

            overlayOutput = new CvSource(
                    "RioVisionOverlay",
                    PixelFormat.kBGR,
                    Constants.Vision.CAMERA_WIDTH,
                    Constants.Vision.CAMERA_HEIGHT,
                    Constants.Vision.CAMERA_FPS);
            overlayServer = new MjpegServer(
                    "RioVisionOverlayServer",
                    Constants.Vision.CAMERA_OVERLAY_STREAM_PORT);
            overlayServer.setSource(overlayOutput);

            // Configure AprilTag detector for 36h11 tag family
            detector = new AprilTagDetector();
            detector.addFamily("tag36h11");
        } catch (Exception ex) {
            System.err.println("[RioVisionThread] FATAL: Camera/detector init failed: " + ex.getMessage());
            cameraDebugInfo.set(cameraDebugInfo.get().withError("INIT_FAILED", ex.getMessage()));
            ex.printStackTrace();
            grayMat.release();
            mat.release();
            return;
        }

        while (!Thread.interrupted()) {
            try {
                // grabFrame blocks until the next frame arrives (or timeout).
                // 500ms timeout prevents blocking forever if camera disconnects.
                long frameTime = cvSink.grabFrame(mat, 0.5);
                if (frameTime == 0) {
                    // Frame grab failed — camera probably disconnected.
                    // Retry on next iteration; main loop sees stale result.
                    consecutiveGrabFailures++;
                    double nowSec = Timer.getFPGATimestamp();
                    String error = cvSink.getError();
                    if (error == null || error.isBlank()) {
                        error = "unknown cscore error";
                    }
                    // After sustained failures, flag camera as disconnected so the
                    // operator knows vision is down (not just a transient glitch).
                    if (consecutiveGrabFailures >= 30 && !disconnectLogged) {
                        System.err.println("[RioVisionThread] CAMERA DISCONNECTED — "
                                + consecutiveGrabFailures + " consecutive grab failures. "
                                + "Check USB connection.");
                        cameraDebugInfo.set(cameraDebugInfo.get()
                                .withError("CAMERA_DISCONNECTED", error));
                        disconnectLogged = true;
                    } else {
                        cameraDebugInfo.set(cameraDebugInfo.get().withError("GRAB_FAILED", error));
                    }
                    if (nowSec - lastGrabErrorLogSec >= 1.0) {
                        System.err.println("[RioVisionThread] grabFrame() failed ("
                                + consecutiveGrabFailures + " consecutive): " + error);
                        lastGrabErrorLogSec = nowSec;
                    }
                    // Clear the latest result so main loop doesn't use stale data
                    latestResult.set(null);
                    continue;
                }

                // Frame received successfully — reset disconnect tracking
                consecutiveGrabFailures = 0;
                if (disconnectLogged) {
                    System.out.println("[RioVisionThread] Camera reconnected — frames resuming.");
                    disconnectLogged = false;
                }

                double timestampSec = Timer.getFPGATimestamp();
                lastFrameTimestampSec.set(timestampSec);
                frameCount++;
                cameraDebugInfo.set(cameraDebugInfo.get().withFrame(frameCount, timestampSec));
                if (!firstFrameLogged) {
                    System.out.println("[RioVisionThread] First camera frame received at t=" + timestampSec);
                    firstFrameLogged = true;
                }

                Mat detectorFrame = VisionSupport.prepareDetectorFrame(mat, grayMat);
                AprilTagDetection[] detections = detector.detect(detectorFrame);
                int[] hubTagIds = getAllianceHubTagIds();
                annotateFrame(mat, detections, hubTagIds);
                overlayOutput.putFrame(mat);
                if (detections.length == 0) {
                    latestResult.set(null);
                    continue;
                }

                // Filter for alliance HUB tags and pick the largest (closest)
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
                    latestResult.set(null);
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
            } catch (Exception ex) {
                // Log but don't crash — keep retrying next frame
                System.err.println("[RioVisionThread] Frame processing error: " + ex.getMessage());
            }
        }

        detector.close();
        if (overlayServer != null) {
            overlayServer.close();
        }
        if (overlayOutput != null) {
            overlayOutput.close();
        }
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

    private static void logEnumeratedUsbCameras(UsbCameraInfo[] cameras) {
        if (cameras.length == 0) {
            System.err.println("[RioVisionThread] No USB cameras enumerated before capture.");
            return;
        }

        for (UsbCameraInfo camera : cameras) {
            System.out.println(
                    "[RioVisionThread] Enumerated USB camera dev=" + camera.dev
                            + " path=" + camera.path
                            + " name=" + camera.name
                            + " vid=0x" + Integer.toHexString(camera.vendorId)
                            + " pid=0x" + Integer.toHexString(camera.productId));
        }
    }

    private static String summarizeUsbCameras(UsbCameraInfo[] cameras) {
        if (cameras.length == 0) {
            return "none";
        }
        StringBuilder summary = new StringBuilder();
        for (int i = 0; i < cameras.length; i++) {
            UsbCameraInfo camera = cameras[i];
            if (i > 0) {
                summary.append(" | ");
            }
            summary.append("dev=").append(camera.dev)
                    .append(' ')
                    .append(camera.name)
                    .append(" @ ")
                    .append(camera.path);
        }
        return summary.toString();
    }

    private static UsbCameraInfo findCameraInfo(UsbCameraInfo[] cameras, int deviceId) {
        for (UsbCameraInfo camera : cameras) {
            if (camera.dev == deviceId) {
                return camera;
            }
        }
        return null;
    }

    private static UsbCameraInfo safeGetCameraInfo(UsbCamera camera) {
        try {
            return camera.getInfo();
        } catch (Exception ex) {
            return null;
        }
    }

    private static void annotateFrame(Mat frame, AprilTagDetection[] detections, int[] hubTagIds) {
        Scalar centerColor = new Scalar(255, 255, 255);
        Scalar hubColor = new Scalar(60, 210, 80);
        Scalar otherColor = new Scalar(255, 180, 60);
        Scalar textColor = new Scalar(255, 255, 255);

        int imageCenterX = Constants.Vision.CAMERA_WIDTH / 2;
        int imageCenterY = Constants.Vision.CAMERA_HEIGHT / 2;
        Imgproc.drawMarker(frame, new Point(imageCenterX, imageCenterY), centerColor, Imgproc.MARKER_CROSS, 18, 1);
        Imgproc.putText(
                frame,
                "RIO TAG OVERLAY",
                new Point(8, 18),
                Imgproc.FONT_HERSHEY_SIMPLEX,
                0.45,
                textColor,
                1);

        for (AprilTagDetection detection : detections) {
            boolean hubTarget = isHubTag(detection.getId(), hubTagIds);
            Scalar color = hubTarget ? hubColor : otherColor;
            Point[] corners = new Point[] {
                new Point(detection.getCornerX(0), detection.getCornerY(0)),
                new Point(detection.getCornerX(1), detection.getCornerY(1)),
                new Point(detection.getCornerX(2), detection.getCornerY(2)),
                new Point(detection.getCornerX(3), detection.getCornerY(3))
            };
            for (int i = 0; i < corners.length; i++) {
                Imgproc.line(frame, corners[i], corners[(i + 1) % corners.length], color, 2);
            }
            Point labelPoint = new Point(detection.getCornerX(0), Math.max(12.0, detection.getCornerY(0) - 6.0));
            Imgproc.putText(
                    frame,
                    (hubTarget ? "HUB " : "TAG ") + detection.getId(),
                    labelPoint,
                    Imgproc.FONT_HERSHEY_SIMPLEX,
                    0.45,
                    color,
                    1);
        }
    }
}
