package org.firstinspires.ftc.teamcode.Tensorflow;


import android.hardware.Camera;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.util.ArrayList;

public class TFODCalc {
    private static float FOCAL_LENGTH = 1.0f;    //in mm
    private static double SENSOR_HEIGHT = 1.0;    //in mm
    private static Camera camera;
    private static ArrayList<ArrayList<Double>> autoAdjustedOffset = new ArrayList<>();

    public static void init() {
        for (int i = 0; i < 100; i++)
            autoAdjustedOffset.add(new ArrayList<>());
    }

    public static double getSavedOffset(int index1, int index2) {
        return autoAdjustedOffset.get(index1).get(index2);
    }

    public static void getPhoneCamConstants() {
        camera = Camera.open();
        Camera.Parameters params = camera.getParameters();
        FOCAL_LENGTH = params.getFocalLength();
        float verticalViewAngle = params.getVerticalViewAngle();
        camera.release();
        SENSOR_HEIGHT = Math.tan(Math.toRadians(verticalViewAngle / 2)) * 2 * FOCAL_LENGTH;  //tan(angle/2) * 2 * focalLength (in degrees)
    }

    public static float getFocalLength() {
        return FOCAL_LENGTH;
    }

    public static double getSensorHeight() {
        return SENSOR_HEIGHT;
    }

    public static void setHardwareProperties(float focalLength, double sensorHeight) {
        FOCAL_LENGTH = focalLength;
        SENSOR_HEIGHT = sensorHeight;
    }

    public static void setHardwareProperties(double verticalFOVAngle, float focalLength) {
        FOCAL_LENGTH = focalLength;
        SENSOR_HEIGHT = Math.tan(Math.toRadians(verticalFOVAngle / 2)) * 2 * FOCAL_LENGTH;
    }

    public static double getDistanceToObj(double objHeightmm, double imgHeightpx, double objHeightpx) {
        double dist = (FOCAL_LENGTH * objHeightmm * imgHeightpx) / (objHeightpx * SENSOR_HEIGHT) / 25.4;   //in inches (mm / 25.4)
        return dist;
    }

    private static double autoAdjustDomain(double xInt, double xAt60, double xIntOffset,
                                           double objWidthPx, int objIndex, double prevCalcOffset) {
        double autoAdjustOutput = 0.0;

        /**
         * Allows up to a certain degree of error. If data is past the acceptable error, then tuning algorithm changes
         * the X-offset of the model quadratic, thus better fitting the data. The algorithm finds the delta
         * between the predicted and actual x-intercepts by finding the difference in x values at a certain point.
         */

        if (xInt - objWidthPx <= xInt + xIntOffset - 207.369) {  //Allows up to -5° of error (- 10)
            double allowedErrorOffset = xAt60 - (xInt + prevCalcOffset - 207.369);    //Calculates allowed error offset (- 10)
            autoAdjustOutput = getDomainOffset(xAt60 - allowedErrorOffset, xIntOffset, objWidthPx); //Gets new, additional offset
        } else if (xInt - objWidthPx >= xInt + xIntOffset + 10.611) {   //Allows up to +10° of error (+ 8.611)
            autoAdjustOutput = getDomainOffset(xInt + 10.611, xIntOffset, objWidthPx);   //Calculates new, additional offset
        }
        return autoAdjustOutput;
    }

    private static double getDomainOffset(double anchorX, double xIntOffset, double objWidthPx) {
        double newOffset = anchorX - objWidthPx;    //Actual offset based on current width of the skystone
        return xIntOffset - newOffset;  //Creates an additional offset that is subtracted from the "old" offset (Delta x-offset)
    }
}