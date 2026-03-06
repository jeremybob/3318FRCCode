package frc.robot.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotSame;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

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
}
