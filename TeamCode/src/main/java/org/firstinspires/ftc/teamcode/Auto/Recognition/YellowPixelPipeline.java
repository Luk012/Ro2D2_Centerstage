package org.firstinspires.ftc.teamcode.Auto.Recognition;

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



public class YellowPixelPipeline extends OpenCvPipeline {
    //backlog of frames to average out to reduce noise

    public String whichSide;

    public boolean hasProcessedFrame = false;
    public int max, imx;
    public int mx = -1;

    ArrayList<double[]> frameList;
    //these are public static to be tuned in dashboard
    public static double strictLowS = 140;
    public static double strictHighS = 255;

    public YellowPixelPipeline() {
        frameList = new ArrayList<>();
    }

    Mat YCbCr = new Mat();
    Mat left_Cb, right_Cb;
    Scalar rectColor = new Scalar(255.0, 0.0, 0.0); // red color in RGB - scalar object for the rectangle color on Camera Stream
    Mat Cb = new Mat();
    public int avg_left, avg_right;

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

        Rect left = new Rect(240,50,80,80);
        Rect right = new Rect(320,50,80,80);
        left_Cb = Cb.submat(left);
        right_Cb = Cb.submat(right);

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
        Scalar lowHSV = new Scalar(20, 100, 100); //values are converted to HSV 160 50 20
        Scalar highHSV = new Scalar(30, 255, 255); //values are converted to HSV

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
        //masked.convertTo(scaledMask, -1, 50 / average.val[1], 0);


        Mat scaledThresh = new Mat();

//        Scalar strictLowHSV = new Scalar(0, 100, 100);
//        Scalar strictHighHSV = new Scalar(0, 255, 255);
        Scalar strictLowHSV = new Scalar(20, 100, 100); //values are converted to HSV 160 50 20
        Scalar strictHighHSV = new Scalar(30, 255, 255); //values are converted to HSV


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

//        Rect leftRect = new Rect(1, 212, 205, 148);
//        Rect centerRect = new Rect(206, 212, 186, 148);
//        Rect rightRect = new Rect(392, 212, 208, 148);
//
//        Imgproc.rectangle(input, leftRect, rectColor, 2);
//        Imgproc.rectangle(input, centerRect, rectColor, 2);
//        Imgproc.rectangle(input, rightRect, rectColor, 2);
//
//        Mat region1_bw = input.submat(leftRect);
//        Mat region2_bw = input.submat(centerRect);
//        Mat region3_bw = input.submat(rightRect);

        Rect left = new Rect(240,50,80,80);
        Rect right = new Rect(320,50,80,80);


        Imgproc.rectangle(input, left, rectColor, 2);
        Imgproc.rectangle(input, right, rectColor, 2);


        Mat left_bw = input.submat(left);
        Mat right_bw = input.submat(right);


        avg_left = (int) Core.countNonZero(left_bw);
        avg_right = (int) Core.countNonZero(right_bw);


        left_bw.release();
        right_bw.release();


        int[] averages = {avg_left,  avg_right};

        for(int i = 0; i < averages.length; i++){
            if(averages[i] > mx){
                mx = averages[i];
                imx = i;
            }
        }

        max = mx;

        if(imx == 0) {
            whichSide = "left";
        }
        else {
            whichSide = " right";
        }




        //whichSide = "lol"+avg2;

        hasProcessedFrame = true;

        return input;
    }
}