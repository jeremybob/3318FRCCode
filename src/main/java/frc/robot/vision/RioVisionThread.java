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

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CameraServerJNI;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.OpenCvLoader;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.UsbCameraInfo;
import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
        Mat mat = null;
        Mat grayMat = null;
        double lastGrabErrorLogSec = Double.NEGATIVE_INFINITY;
        boolean firstFrameLogged = false;
        long frameCount = 0;
        int consecutiveGrabFailures = 0;
        boolean disconnectLogged = false;

        try {
            OpenCvLoader.forceLoad();
            mat = new Mat();
            grayMat = new Mat();

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
            AprilTagDetector.Config detectorConfig = detector.getConfig();
            detectorConfig.numThreads = Constants.Vision.APRILTAG_NUM_THREADS;
            detectorConfig.quadDecimate = Constants.Vision.APRILTAG_QUAD_DECIMATE;
            detectorConfig.quadSigma = Constants.Vision.APRILTAG_QUAD_SIGMA;
            detectorConfig.refineEdges = true;
            detectorConfig.decodeSharpening = Constants.Vision.APRILTAG_DECODE_SHARPENING;
            detectorConfig.debug = false;
            detector.setConfig(detectorConfig);

            AprilTagDetector.QuadThresholdParameters quadThresholds = detector.getQuadThresholdParameters();
            quadThresholds.minClusterPixels = Constants.Vision.APRILTAG_MIN_CLUSTER_PIXELS;
            quadThresholds.maxNumMaxima = Constants.Vision.APRILTAG_MAX_NUM_MAXIMA;
            quadThresholds.criticalAngle = Constants.Vision.APRILTAG_CRITICAL_ANGLE_RAD;
            quadThresholds.maxLineFitMSE = Constants.Vision.APRILTAG_MAX_LINE_FIT_MSE;
            quadThresholds.minWhiteBlackDiff = Constants.Vision.APRILTAG_MIN_WHITE_BLACK_DIFF;
            quadThresholds.deglitch = Constants.Vision.APRILTAG_DEGLITCH;
            detector.setQuadThresholdParameters(quadThresholds);
            detector.addFamily("tag36h11");
        } catch (Throwable ex) {
            System.err.println("[RioVisionThread] FATAL: Camera/detector init failed: " + ex.getMessage());
            cameraDebugInfo.set(cameraDebugInfo.get().withError("INIT_FAILED", formatThrowable(ex)));
            ex.printStackTrace();
            if (grayMat != null) {
                grayMat.release();
            }
            if (mat != null) {
                mat.release();
            }
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
                VisionSupport.HubCenterEstimate hubEstimate = estimateHubCenter(detections, hubTagIds);
                annotateFrame(mat, detections, hubTagIds, hubEstimate);
                overlayOutput.putFrame(mat);
                if (hubEstimate == null) {
                    latestResult.set(null);
                    publishNoTargetVisionDebug();
                    continue;
                }

                double hubYawDeg = pixelToYawDeg(hubEstimate.centerX());
                double hubPitchDeg = pixelToPitchDeg(hubEstimate.centerY());
                double bestTagYawDeg = pixelToYawDeg(hubEstimate.bestTagCenterX());
                double bestTagPitchDeg = pixelToPitchDeg(hubEstimate.bestTagCenterY());
                publishVisionDebug(hubEstimate, hubYawDeg, hubPitchDeg, bestTagYawDeg, bestTagPitchDeg);

                latestResult.set(new VisionResult(
                        hubEstimate.bestTagId(),
                        hubYawDeg,
                        hubPitchDeg,
                        hubEstimate.bestTagPixelHeight(),
                        timestampSec,
                        bestTagYawDeg,
                        bestTagPitchDeg,
                        hubEstimate.hubTagCount(),
                        hubEstimate.hubFaceCount(),
                        hubEstimate.hubSpanPx()));
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

    private static VisionSupport.HubCenterEstimate estimateHubCenter(
            AprilTagDetection[] detections,
            int[] hubTagIds) {
        if (detections.length == 0) {
            return null;
        }

        AprilTagDetection bestHubDetection = null;
        double bestPixelHeight = 0.0;
        List<AprilTagDetection> filteredHubDetections = new ArrayList<>();

        for (AprilTagDetection detection : detections) {
            if (!isHubTag(detection.getId(), hubTagIds)) {
                continue;
            }
            filteredHubDetections.add(detection);
            double pixelHeight = tagPixelHeight(detection);
            if (bestHubDetection == null || pixelHeight > bestPixelHeight) {
                bestHubDetection = detection;
                bestPixelHeight = pixelHeight;
            }
        }

        if (bestHubDetection == null) {
            return null;
        }

        int[] selectedHubTagIds = hubTagIds != null ? hubTagIds : hubTagIdsForTag(bestHubDetection.getId());
        if (selectedHubTagIds == null) {
            return null;
        }

        List<VisionSupport.HubTagObservation> observations = new ArrayList<>();
        for (AprilTagDetection detection : filteredHubDetections) {
            if (!isHubTag(detection.getId(), selectedHubTagIds)) {
                continue;
            }
            observations.add(new VisionSupport.HubTagObservation(
                    detection.getId(),
                    hubFaceIndex(detection.getId(), selectedHubTagIds),
                    tagCenterX(detection),
                    tagCenterY(detection),
                    tagPixelHeight(detection)));
        }
        return VisionSupport.estimateHubCenter(
                observations,
                Constants.Vision.CAMERA_WIDTH / 2.0,
                Constants.Vision.SINGLE_TAG_CENTER_BIAS_PX_PER_TAG_HEIGHT);
    }

    private static int[] hubTagIdsForTag(int fiducialId) {
        if (isHubTag(fiducialId, Constants.Vision.RED_HUB_TAG_IDS)) {
            return Constants.Vision.RED_HUB_TAG_IDS;
        }
        if (isHubTag(fiducialId, Constants.Vision.BLUE_HUB_TAG_IDS)) {
            return Constants.Vision.BLUE_HUB_TAG_IDS;
        }
        return null;
    }

    private static int hubFaceIndex(int fiducialId, int[] hubTagIds) {
        if (hubTagIds == null) {
            return -1;
        }
        for (int i = 0; i + 1 < hubTagIds.length; i += 2) {
            if (hubTagIds[i] == fiducialId || hubTagIds[i + 1] == fiducialId) {
                return i / 2;
            }
        }
        return -1;
    }

    private static double tagCenterX(AprilTagDetection detection) {
        return (detection.getCornerX(0) + detection.getCornerX(1)
                + detection.getCornerX(2) + detection.getCornerX(3)) / 4.0;
    }

    private static double tagCenterY(AprilTagDetection detection) {
        return (detection.getCornerY(0) + detection.getCornerY(1)
                + detection.getCornerY(2) + detection.getCornerY(3)) / 4.0;
    }

    private static double pixelToYawDeg(double centerX) {
        double imageCenterX = Constants.Vision.CAMERA_WIDTH / 2.0;
        return (centerX - imageCenterX) / imageCenterX
                * (Constants.Vision.HORIZONTAL_FOV_DEG / 2.0);
    }

    private static double pixelToPitchDeg(double centerY) {
        double imageCenterY = Constants.Vision.CAMERA_HEIGHT / 2.0;
        return (imageCenterY - centerY) / imageCenterY
                * (Constants.Vision.VERTICAL_FOV_DEG / 2.0);
    }

    private static void publishNoTargetVisionDebug() {
        SmartDashboard.putString("Vision/Estimator", "NONE");
        SmartDashboard.putNumber("Vision/TargetTagId", -1);
        SmartDashboard.putNumber("Vision/HubTagCount", 0);
        SmartDashboard.putNumber("Vision/HubFaceCount", 0);
        SmartDashboard.putNumber("Vision/HubSpanPx", 0.0);
        SmartDashboard.putNumber("Vision/BestTagYawDeg", Double.NaN);
        SmartDashboard.putNumber("Vision/BestTagPitchDeg", Double.NaN);
        SmartDashboard.putNumber("Vision/HubCenterYawDeg", Double.NaN);
        SmartDashboard.putNumber("Vision/HubCenterPitchDeg", Double.NaN);
        SmartDashboard.putNumber("Vision/HubCenterOffsetPx", Double.NaN);
        SmartDashboard.putNumber("Vision/HubYawOffsetDeg", Double.NaN);
    }

    private static void publishVisionDebug(
            VisionSupport.HubCenterEstimate hubEstimate,
            double hubYawDeg,
            double hubPitchDeg,
            double bestTagYawDeg,
            double bestTagPitchDeg) {
        String estimator = hubEstimate.hubFaceCount() > 1
                ? "MULTI_FACE"
                : hubEstimate.hubTagCount() > 1 ? "PAIR_MIDPOINT" : "SINGLE_TAG";
        SmartDashboard.putString("Vision/Estimator", estimator);
        SmartDashboard.putNumber("Vision/TargetTagId", hubEstimate.bestTagId());
        SmartDashboard.putNumber("Vision/HubTagCount", hubEstimate.hubTagCount());
        SmartDashboard.putNumber("Vision/HubFaceCount", hubEstimate.hubFaceCount());
        SmartDashboard.putNumber("Vision/HubSpanPx", hubEstimate.hubSpanPx());
        SmartDashboard.putNumber("Vision/BestTagYawDeg", bestTagYawDeg);
        SmartDashboard.putNumber("Vision/BestTagPitchDeg", bestTagPitchDeg);
        SmartDashboard.putNumber("Vision/HubCenterYawDeg", hubYawDeg);
        SmartDashboard.putNumber("Vision/HubCenterPitchDeg", hubPitchDeg);
        SmartDashboard.putNumber("Vision/HubCenterOffsetPx", hubEstimate.centerOffsetFromBestPx());
        SmartDashboard.putNumber("Vision/HubYawOffsetDeg", hubYawDeg - bestTagYawDeg);
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

    private static String formatThrowable(Throwable throwable) {
        String message = throwable.getMessage();
        if (message == null || message.isBlank()) {
            return throwable.getClass().getSimpleName();
        }
        return throwable.getClass().getSimpleName() + ": " + message;
    }

    private static void annotateFrame(
            Mat frame,
            AprilTagDetection[] detections,
            int[] hubTagIds,
            VisionSupport.HubCenterEstimate hubEstimate) {
        Scalar centerColor = new Scalar(255, 255, 255);
        Scalar hubColor = new Scalar(60, 210, 80);
        Scalar otherColor = new Scalar(255, 180, 60);
        Scalar textColor = new Scalar(255, 255, 255);
        Scalar bestTagColor = new Scalar(0, 200, 255);
        Scalar hubCenterColor = new Scalar(255, 255, 0);

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

        String summary = "HUB CTR --";
        String coverage = "tags 0 faces 0";
        if (hubEstimate != null) {
            Point bestPoint = new Point(hubEstimate.bestTagCenterX(), hubEstimate.bestTagCenterY());
            Point hubCenterPoint = new Point(hubEstimate.centerX(), hubEstimate.centerY());
            Imgproc.drawMarker(frame, bestPoint, bestTagColor, Imgproc.MARKER_DIAMOND, 12, 1);
            Imgproc.drawMarker(frame, hubCenterPoint, hubCenterColor, Imgproc.MARKER_TILTED_CROSS, 24, 2);
            Imgproc.circle(frame, hubCenterPoint, 10, hubCenterColor, 2);
            Imgproc.line(frame, new Point(imageCenterX, imageCenterY), hubCenterPoint, hubCenterColor, 1);
            Imgproc.line(frame, bestPoint, hubCenterPoint, bestTagColor, 1);

            double hubYawDeg = pixelToYawDeg(hubEstimate.centerX());
            double bestYawDeg = pixelToYawDeg(hubEstimate.bestTagCenterX());
            summary = "HUB " + formatSigned(hubYawDeg) + " deg"
                    + "  BEST " + formatSigned(bestYawDeg) + " deg";
            coverage = "tags " + hubEstimate.hubTagCount()
                    + " faces " + hubEstimate.hubFaceCount()
                    + " span " + Math.round(hubEstimate.hubSpanPx()) + " px";
        }

        Imgproc.putText(frame, summary, new Point(8, 36), Imgproc.FONT_HERSHEY_SIMPLEX, 0.42, textColor, 1);
        Imgproc.putText(frame, coverage, new Point(8, 54), Imgproc.FONT_HERSHEY_SIMPLEX, 0.42, textColor, 1);
    }

    private static String formatSigned(double value) {
        if (!Double.isFinite(value)) {
            return "--";
        }
        return String.format("%+.1f", value);
    }
}
