package frc.robot.vision;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

public final class VisionSupport {

    private VisionSupport() {}

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
}
