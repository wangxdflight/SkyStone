package org.firstinspires.ftc.teamcode.autonomous.vision;

//import org.corningrobotics.enderbots.endercv.OpenCVPipeline;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import org.openftc.easyopencv.OpenCvPipeline;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import com.qualcomm.robotcore.util.RobotLog;

import static org.opencv.core.CvType.CV_8UC1;
import static org.opencv.core.CvType.CV_8UC3;

public class CalibratingPipeline extends OpenCvPipeline {
    public Mat rgbaCapture;
    private boolean captureFlag;

    private Threshold threshold;

    private Mat hsv = new Mat();
    private Mat mask = new Mat();

    private Mat hierarchy = new Mat();
    private List<MatOfPoint> contours = new ArrayList<>();

    public void setMask(Mat mask) {
        Imgproc.cvtColor(mask, mask, Imgproc.COLOR_BGR2GRAY);
        Imgproc.resize(mask, mask, new Size(1280, 720));
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
    }

    @Override
    public Mat processFrame(Mat rgba) {//}, Mat gray) {
        RobotLog.i("processFrame: "+rgba.total());
        if (captureFlag) {
            rgbaCapture = rgba.clone();
            captureFlag = false;
        }
        if (threshold == null) {
            Imgproc.drawContours(rgba, contours, -1, new Scalar(0, 0, 255), 3);
        }
        else {
            Imgproc.cvtColor(rgba, hsv, Imgproc.COLOR_RGB2HSV);
            threshold.threshold(hsv, mask);

            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

            int largestContourIndex = 0;
            double largestContourSize = 0;

            for (int i = 0; i < contours.size(); i++) {
                if (Imgproc.contourArea(contours.get(i)) > largestContourSize) {
                    largestContourIndex = i;
                    largestContourSize = Imgproc.contourArea(contours.get(i));
                }
            }

            Imgproc.drawContours(rgba, contours, largestContourIndex, new Scalar(0, 255, 0));
        }
        return rgba;
    }

    public void capture() {
        captureFlag = true;
    }

    public void setThreshold(Threshold threshold) {
        this.threshold = threshold;
    }
}
