package org.firstinspires.ftc.teamcode.subsystems.pipelines;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.stealthrobotics.library.Alliance;

public class PropProcessor implements VisionProcessor {
    Mat testMat = new Mat();
    Mat highMat = new Mat();
    Mat lowMat = new Mat();
    Mat finalMat = new Mat();

    double redThreshold = 0.5;

    String outStr = "left";
    //TODO: tune these values

    static final Point LeftPoint1 = new Point(0,160);
    static final Point LeftPoint2 = new Point(150,290);
    static final Rect LEFT_RECTANGLE = new Rect(
            LeftPoint1,
            LeftPoint2
    );
    static final Point CenterPoint1 = new Point(200,160);
    static final Point CenterPoint2 = new Point(350,270);
    static final Rect CENTER_RECT = new Rect(
            CenterPoint1,
            CenterPoint2
    );
    static final Point RightPoint1 = new Point(450,160);
    static final Point RightPoint2 = new Point(640,270);
    static final Rect RIGHT_RECTANGLE = new Rect(
            RightPoint1,
            RightPoint2
    );

    Scalar lowHSVColorUpper;
    Scalar highHSVColorLower;
    Scalar lowHSVColorLower;
    Scalar highHSVColorUpper;

    public PropProcessor(Alliance alliance){
        //sets the color thresholds based on alliance
        if(alliance == Alliance.RED){
            lowHSVColorLower = new Scalar(0, 170, 164); //beginning of red
            lowHSVColorUpper = new Scalar(12.8, 255, 255);

            highHSVColorLower = new Scalar(138, 90, 160); //end of red
            highHSVColorUpper = new Scalar(255, 255, 255);
        }

        else if(alliance == Alliance.BLUE){
            lowHSVColorLower = new Scalar(100, 100, 100); //blue thresholding
            lowHSVColorUpper = new Scalar(160, 255, 255);

            highHSVColorLower = new Scalar(160, 255, 255);
            highHSVColorUpper = new Scalar(160, 255, 255);
        }
    }
    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Mat drawMat = frame;
        //convert to hsv for thresholding
        Imgproc.cvtColor(frame, testMat, Imgproc.COLOR_RGB2HSV);


        //maps white to everything in color range and black to everything out of color range
        Core.inRange(testMat, lowHSVColorLower, lowHSVColorUpper, lowMat);
        Core.inRange(testMat, highHSVColorLower, highHSVColorUpper, highMat);

        testMat.release();

        //combines ranges since red is split between 0 and 180
        Core.bitwise_or(lowMat, highMat, finalMat);

        lowMat.release();
        highMat.release();

        //gets the average of the pixels in each rectangle
        double leftBox = Core.sumElems(finalMat.submat(LEFT_RECTANGLE)).val[0] / (LEFT_RECTANGLE.width * LEFT_RECTANGLE.height);
        double rightBox = Core.sumElems(finalMat.submat(RIGHT_RECTANGLE)).val[0] / (RIGHT_RECTANGLE.width * RIGHT_RECTANGLE.height);
        double centerBox = Core.sumElems(finalMat.submat(CENTER_RECT)).val[0] / (CENTER_RECT.width * CENTER_RECT.height);

        //finds max of 3 areas and sets the output string to the corresponding area
        double max = Math.max(Math.max(leftBox, rightBox), centerBox);
        if(leftBox == max) outStr = "left";
        else if(rightBox == max) outStr = "right";
        else outStr = "center";

        //Imgproc.line(drawMat, new Point(220, 0), new Point(220, 480), new Scalar(0, 255, 0), 4);
        //Imgproc.line(drawMat, new Point(450, 0), new Point(450, 480), new Scalar(0, 255, 0), 4);
        Imgproc.rectangle(drawMat,LeftPoint1,LeftPoint2,new Scalar(0,255,0),4);
        Imgproc.rectangle(drawMat,CenterPoint1,CenterPoint2,new Scalar(0,255,0),4);
        Imgproc.rectangle(drawMat,RightPoint1,RightPoint2,new Scalar(0,255,0),4);

        drawMat.copyTo(frame);

        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
    public String getOutStr(){
        return outStr;
    }
}