package frc.robot.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotSame;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;

class VisionSupportTest {

    @BeforeAll
    static void loadOpenCv() {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
    }

    @Test
    void prepareDetectorFrameConvertsColorFramesToGrayscale() {
        Mat colorFrame = new Mat(2, 2, CvType.CV_8UC3, new Scalar(10, 20, 30));
        Mat grayBuffer = new Mat();
        try {
            Mat detectorFrame = VisionSupport.prepareDetectorFrame(colorFrame, grayBuffer);
            assertEquals(1, detectorFrame.channels());
            assertNotSame(colorFrame, detectorFrame);
        } finally {
            colorFrame.release();
            grayBuffer.release();
        }
    }

    @Test
    void prepareDetectorFrameReturnsOriginalGrayscaleFrame() {
        Mat grayFrame = new Mat(2, 2, CvType.CV_8UC1, new Scalar(42));
        Mat grayBuffer = new Mat();
        try {
            Mat detectorFrame = VisionSupport.prepareDetectorFrame(grayFrame, grayBuffer);
            assertSame(grayFrame, detectorFrame);
            assertEquals(1, detectorFrame.channels());
        } finally {
            grayFrame.release();
            grayBuffer.release();
        }
    }

    @Test
    void cameraConnectionUsesRecentFrameHeartbeat() {
        assertTrue(VisionSupport.isCameraConnected(12.0, 10.5, 2.0));
        assertFalse(VisionSupport.isCameraConnected(12.0, 9.9, 2.0));
        assertFalse(VisionSupport.isCameraConnected(12.0, Double.NaN, 2.0));
        assertFalse(VisionSupport.isCameraConnected(12.0, null, 2.0));
    }

    @Test
    void estimateHubCenterAveragesVisibleFacesNotIndividualTags() {
        VisionSupport.HubCenterEstimate estimate = VisionSupport.estimateHubCenter(List.of(
                new VisionSupport.HubTagObservation(18, 0, 100.0, 200.0, 40.0),
                new VisionSupport.HubTagObservation(19, 0, 120.0, 200.0, 42.0),
                new VisionSupport.HubTagObservation(20, 1, 220.0, 180.0, 30.0)));

        assertEquals(165.0, estimate.centerX(), 1e-9);
        assertEquals(190.0, estimate.centerY(), 1e-9);
        assertEquals(19, estimate.bestTagId());
        assertEquals(3, estimate.hubTagCount());
        assertEquals(2, estimate.hubFaceCount());
        assertEquals(120.0, estimate.hubSpanPx(), 1e-9);
        assertEquals(45.0, estimate.centerOffsetFromBestPx(), 1e-9);
    }

    @Test
    void estimateHubCenterReturnsNullWhenNoValidObservationsExist() {
        VisionSupport.HubCenterEstimate estimate = VisionSupport.estimateHubCenter(List.of(
                new VisionSupport.HubTagObservation(18, 0, Double.NaN, 200.0, 40.0),
                new VisionSupport.HubTagObservation(19, 0, 120.0, 200.0, 0.0)));

        assertNull(estimate);
    }
}
