package org.firstinspires.ftc.teamcode.commands.subsystems.pipelines;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class TeamPropDetection extends OpenCvPipeline {
    //The bounds of each cam region
    private static final Point Size = new Point(20,20);

    private static final Rect[] cameras = {
            new Rect(new Point(40,20), Size),
            new Rect(new Point(80,20), Size),
            new Rect(new Point(120,20), Size)
    };

    private volatile int conePosition = 0;

    @Override
    public Mat processFrame(Mat input) {
        //figures out which has the most red/blue
        Scalar[] camRegions = new Scalar[3];
        for (int i = 0; i < 3; i++){
            camRegions[i] = Core.sumElems(input.submat(cameras[i]));
        }
        double maxColor = Math.max(Math.max(camRegions[0].val[0],camRegions[1].val[0]),camRegions[2].val[0]);
        //draws boxes and sets value
        for(int i = 0; i < 3; i++){
            Point corner = new Point(cameras[i].x, cameras[i].y);
            Point corner2 = new Point(cameras[i].x + cameras[i].width, cameras[i].y + cameras[i].height);
            if(maxColor == camRegions[i].val[0]){
                //sets value
                conePosition = i;
                Imgproc.rectangle(
                        input,
                        corner,
                        corner2,
                        new Scalar(0,255,0),
                        2
                );
            }
            else{
                Imgproc.rectangle(
                        input,
                        corner,
                        corner2,
                        new Scalar(0,0,0),
                        2
                );
            }
        }
        return input;
    }
    public int GetConePosition(){
        return conePosition;
    }
}
