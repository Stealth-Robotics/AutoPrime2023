package org.firstinspires.ftc.teamcode.subsystems.pipelines;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.openftc.easyopencv.OpenCvPipeline;

public class TeamPropDetection extends OpenCvPipeline {
    //The bounds of each cam region
    private static final Point TopLeft0 = new Point(40,20);
    private static final Point BottomRight0 = new Point(20 + TopLeft0.x,20 + TopLeft0.y);

    private static final Point TopLeft1 = new Point(80,20);
    private static final Point BottomRight1 = new Point(20 + TopLeft1.x,20 + TopLeft1.y);

    private static final Point TopLeft2 = new Point(120,20);
    private static final Point BottomRight2 = new Point(20 + TopLeft2.x,20 + TopLeft2.y);

    private static final Rect[] cameras = {
            new Rect(TopLeft0,BottomRight0),
            new Rect(TopLeft1,BottomRight1),
            new Rect(TopLeft2,BottomRight2)
    };

    private volatile int conePosition = 0;

    @Override
    public Mat processFrame(Mat input) {
        //TODO
        //Figure out what the most blue/red camera is to figure out what to do during auto


        return input;
    }
}
