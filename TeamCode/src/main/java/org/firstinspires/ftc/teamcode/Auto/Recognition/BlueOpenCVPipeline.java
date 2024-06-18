package org.firstinspires.ftc.teamcode.Auto.Recognition;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

/**
 * Scalar CLASS is used to represent BGR color values.
 * Scalar(a, b, c) - Blue = a, Green = b, Red = c
 *
 * Mat CLASS objects are used for storing frames.
 *
 * YCbCr COLOR SPACE separates the luminance(Y) and chrominance(CbCr) components of an image.
 * The Y CHANNEL represents the brightness of the color,
 * while the Cb and Cr CHANNELS represent the blue and red color differences.
 *
 * The point(0, 0) corresponds to the left-corner of the image - the ORIGIN
 * Moving down and to the right, both x, y values increase
 * X = COLUMN NUMBER
 * Y = RAW NUMBER
 *
 * lowHSV and highHSV are used to track an object of a particular color
 */


@Config
public class BlueOpenCVPipeline extends OpenCvPipeline {
    //backlog of frames to average out to reduce noise

    public String whichSide = null;

    public boolean hasProcessedFrame = false;
    public int max;

    ArrayList<double[]> frameList;
    //these are public static to be tuned in dashboard
    public static double strictLowS = 140; //NEVER used
    public static double strictHighS = 255; //NEVER used

    public static double lowS  = 75;
    public static double highS = 255;

    public static double lowV  = 75;
    public static double highV = 255;

    public BlueOpenCVPipeline() {
        frameList = new ArrayList<>();
    }

    Mat YCbCr = new Mat();
    Mat region1_Cb, region2_Cb, region3_Cb;
    Mat outPut = new Mat();
    Scalar rectColor = new Scalar(0.0, 0.0, 255.0); // blue color in RGB
    Scalar rectColorFound = new Scalar(255.0, 100.0, 100.0); // light color
    Mat Cb = new Mat();
    public int avg1, avg2, avg3;

    // read frame
    void inputToCb(Mat input)
    {
        Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(YCbCr, Cb, 2); // 2 for red CHANNEL and 0 for blue CHANNEL
    }

    // which side of the tile the team prop is
    public String getWhichSide() {
        return whichSide;
    }
    @Override
    public void init(Mat firstFrame) {
        /*
         * We need to call this in order to make sure the 'Cb'
         * object is initialized, so that the submats we make
         * will still be linked to it on subsequent frames. (If
         * the object were to only be initialized in processFrame,
         * then the submats would become delinked because the backing
         * buffer would be re-allocated the first time a real frame
         * was crunched)
         */
        inputToCb(firstFrame);

        /*
         * Submats are a persistent reference to a region of the parent
         * buffer. Any changes to the child affect the parent, and the
         * reverse also holds true.
         */
//        Rect leftRect = new Rect(1, 91, 294, 269);
//        Rect centerRect = new Rect(295, 91, 226, 269);
//        Rect rightRect = new Rect(521, 91, 119, 269); asta e ala bun

        Rect leftRect = new Rect(150, 160, 154, 200);
        Rect centerRect = new Rect(315, 160, 160, 200);
        Rect rightRect = new Rect(531, 160, 109, 200);

        region1_Cb = Cb.submat(leftRect);
        region2_Cb = Cb.submat(centerRect);
        region3_Cb = Cb.submat(rightRect);
    }

    @Override
    public Mat processFrame(Mat input) {
        Mat mat = new Mat();

        //mat turns into HSV value
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        if (mat.empty()) {
            return input;
        }

        // lenient bounds will filter out near yellow, this should filter out all near yellow things(tune this if needed)
//        Scalar lowHSV = new Scalar(0, 100, 100);
//        Scalar highHSV = new Scalar(0, 255, 255);
        Scalar lowHSV = new Scalar(90, lowS, lowV); //values are converted to HSV 90 50 50
        Scalar highHSV = new Scalar(150, highS, highV); //values are converted to HSV 130 255 255

        Mat thresh = new Mat();

        // Get a black and white image of yellow objects
        Core.inRange(mat, lowHSV, highHSV, thresh);

        Mat masked = new Mat();
        //color the white portion of thresh in with HSV from mat
        //output into masked
        Core.bitwise_and(mat, mat, masked, thresh);
        //calculate average HSV values of the white thresh values
        Scalar average = Core.mean(masked, thresh);

        Mat scaledMask = new Mat();
        //scale the average saturation to 150
        //masked.convertTo(scaledMask, -1, 150 / average.val[1], 0);


        Mat scaledThresh = new Mat();

//        Scalar strictLowHSV = new Scalar(0, 100, 100);
//        Scalar strictHighHSV = new Scalar(0, 255, 255);
        Scalar strictLowHSV = new Scalar(90, lowS, lowV); //values are converted to HSV 90 100 100
        Scalar strictHighHSV = new Scalar(150, highS, highV); //values are converted to HSV 130 255 255


        //apply strict HSV filter onto scaledMask to get rid of any yellow other than pole
        Core.inRange(masked, strictLowHSV, strictHighHSV, scaledThresh);

        Mat finalMask = new Mat();
        //color in scaledThresh with HSV, output into finalMask(only useful for showing result)(you can delete)
        Core.bitwise_and(mat, mat, finalMask, scaledThresh);

        Mat edges = new Mat();
        //detect edges(only useful for showing result)(you can delete)
        Imgproc.Canny(scaledThresh, edges, 100, 200);

        //contours, apply post processing to information
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        //find contours, input scaledThresh because it has hard edges
        Imgproc.findContours(scaledThresh, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        //list of frames to reduce inconsistency, not too many so that it is still real-time, change the number from 5 if you want
        if (frameList.size() > 5) {
            frameList.remove(0);
        }


        //release all the data
        input.release();
        scaledThresh.copyTo(input);
        scaledThresh.release();
        scaledMask.release();
        mat.release();
        masked.release();
        edges.release();
        thresh.release();
        finalMask.release();

        //change the return to whatever mat you want
        //for example, if I want to look at the lenient thresh:
        // return thresh;
        // note that you must not do thresh.release() if you want to return thresh
        // you also need to release the input if you return thresh(release as much as possible)

        Rect leftRect = new Rect(150, 160, 154, 200);
        Rect centerRect = new Rect(315, 160, 160, 200);
        Rect rightRect = new Rect(531, 160, 109, 200);

        Imgproc.rectangle(input, leftRect, rectColor, 2);
        Imgproc.rectangle(input, centerRect, rectColor, 2);
        Imgproc.rectangle(input, rightRect, rectColor, 2);

        Mat region1_bw = input.submat(leftRect);
        Mat region2_bw = input.submat(centerRect);
        Mat region3_bw = input.submat(rightRect);

        avg1 = (int) Core.countNonZero(region1_bw);
        avg2 = (int) Core.countNonZero(region2_bw);
        avg3 = (int) Core.countNonZero(region3_bw);

        region1_bw.release();
        region2_bw.release();
        region3_bw.release();

        int maxOneTwo = Math.max(avg1, avg2);
        max = Math.max(maxOneTwo, avg3);

        if(max == avg1) {
            whichSide = "left";
        }
        else if(max == avg2) {
            whichSide = "center";
        }
        else if(max == avg3) {
            whichSide = "right";
        }



        //whichSide = "lol"+avg2;

        hasProcessedFrame = true;

        return input;
    }
}