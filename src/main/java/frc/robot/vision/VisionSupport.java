package frc.robot.vision;

import java.util.Collection;
import java.util.LinkedHashMap;
import java.util.Map;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import frc.robot.Constants;

public final class VisionSupport {

    private VisionSupport() {}

    public record HubTagObservation(
            int tagId,
            int faceIndex,
            double centerX,
            double centerY,
            double pixelHeight) {}

    public record HubCenterEstimate(
            int bestTagId,
            double centerX,
            double centerY,
            double bestTagCenterX,
            double bestTagCenterY,
            double bestTagPixelHeight,
            int hubTagCount,
            int hubFaceCount,
            double hubSpanPx) {

        public double centerOffsetFromBestPx() {
            return centerX - bestTagCenterX;
        }
    }

    public static Mat prepareDetectorFrame(Mat capturedFrame, Mat grayFrame) {
        int channels = capturedFrame.channels();
        if (channels <= 1) {
            return capturedFrame;
        }
        if (channels == 4) {
            Imgproc.cvtColor(capturedFrame, grayFrame, Imgproc.COLOR_BGRA2GRAY);
            return grayFrame;
        }
        Imgproc.cvtColor(capturedFrame, grayFrame, Imgproc.COLOR_BGR2GRAY);
        return grayFrame;
    }

    public static boolean isCameraConnected(
            double nowSec,
            Double lastFrameTimestampSec,
            double heartbeatTimeoutSec) {
        return lastFrameTimestampSec != null
                && Double.isFinite(lastFrameTimestampSec)
                && nowSec - lastFrameTimestampSec <= heartbeatTimeoutSec;
    }

    public static double calibrateDistanceM(double rawDistanceM) {
        if (!Double.isFinite(rawDistanceM)) {
            return Double.NaN;
        }
        double calibrated = rawDistanceM * Constants.Vision.DISTANCE_CALIBRATION_SCALE
                + Constants.Vision.DISTANCE_CALIBRATION_OFFSET_M;
        return Math.max(0.1, calibrated);
    }

    public static HubCenterEstimate estimateHubCenter(Collection<HubTagObservation> observations) {
        return estimateHubCenter(observations, Double.NaN, 0.0);
    }

    public static HubCenterEstimate estimateHubCenter(
            Collection<HubTagObservation> observations,
            double imageCenterX,
            double singleTagCenterBiasPxPerTagHeight) {
        if (observations == null || observations.isEmpty()) {
            return null;
        }

        HubTagObservation bestObservation = null;
        double minX = Double.POSITIVE_INFINITY;
        double maxX = Double.NEGATIVE_INFINITY;
        int tagCount = 0;
        Map<Integer, FaceAccumulator> faces = new LinkedHashMap<>();

        for (HubTagObservation observation : observations) {
            if (observation == null
                    || !Double.isFinite(observation.centerX())
                    || !Double.isFinite(observation.centerY())
                    || !Double.isFinite(observation.pixelHeight())
                    || observation.pixelHeight() <= 0.0) {
                continue;
            }

            tagCount++;
            if (bestObservation == null || observation.pixelHeight() > bestObservation.pixelHeight()) {
                bestObservation = observation;
            }

            minX = Math.min(minX, observation.centerX());
            maxX = Math.max(maxX, observation.centerX());

            int faceKey = observation.faceIndex() >= 0 ? observation.faceIndex() : observation.tagId();
            faces.computeIfAbsent(faceKey, ignored -> new FaceAccumulator()).add(observation);
        }

        if (bestObservation == null || faces.isEmpty()) {
            return null;
        }

        double summedFaceCenterX = 0.0;
        double summedFaceCenterY = 0.0;
        for (FaceAccumulator accumulator : faces.values()) {
            summedFaceCenterX += accumulator.centerX();
            summedFaceCenterY += accumulator.centerY();
        }

        double hubCenterX = summedFaceCenterX / faces.size();
        double hubCenterY = summedFaceCenterY / faces.size();
        if (faces.size() == 1
                && tagCount == 1
                && Double.isFinite(imageCenterX)
                && Double.isFinite(singleTagCenterBiasPxPerTagHeight)
                && singleTagCenterBiasPxPerTagHeight > 0.0) {
            double towardImageCenter = Math.signum(imageCenterX - hubCenterX);
            if (towardImageCenter != 0.0) {
                double maxBiasPx = Math.abs(imageCenterX - hubCenterX);
                double requestedBiasPx = bestObservation.pixelHeight() * singleTagCenterBiasPxPerTagHeight;
                double appliedBiasPx = Math.min(maxBiasPx, Math.max(0.0, requestedBiasPx));
                hubCenterX += towardImageCenter * appliedBiasPx;
            }
        }
        double hubSpanPx = tagCount >= 2 ? maxX - minX : 0.0;

        return new HubCenterEstimate(
                bestObservation.tagId(),
                hubCenterX,
                hubCenterY,
                bestObservation.centerX(),
                bestObservation.centerY(),
                bestObservation.pixelHeight(),
                tagCount,
                faces.size(),
                hubSpanPx);
    }

    private static final class FaceAccumulator {
        private double sumX;
        private double sumY;
        private int samples;

        void add(HubTagObservation observation) {
            sumX += observation.centerX();
            sumY += observation.centerY();
            samples++;
        }

        double centerX() {
            return samples > 0 ? sumX / samples : Double.NaN;
        }

        double centerY() {
            return samples > 0 ? sumY / samples : Double.NaN;
        }
    }
}
