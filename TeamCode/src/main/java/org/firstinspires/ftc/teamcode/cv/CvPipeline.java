package org.firstinspires.ftc.teamcode.cv;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

/**
 * Simple pipeline that crops image to get color values
 * @author Peter Tvedt (Supersonics 124 2022/2023)
 */
public class CvPipeline extends OpenCvPipeline {
    List<Mat> channels = new ArrayList<>(3);
    private Mat bgrInput = new Mat();
    private Mat output = new Mat();
    private Mat submat = new Mat();
    private int parkingZone = -1;
    double x1 = 0, y1 = 0;
    double x2 = 1, y2 = 1;

    public CvPipeline(double x1, double y1, double x2, double y2) {
        super();
        this.x1 = x1;
        this.y1 = y1;
        this.x2 = x2;
        this.y2 = y2;
    }

    public CvPipeline() {
        this(0, 0, 1, 1);
    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, bgrInput, Imgproc.COLOR_RGBA2BGR);
        submat = bgrInput.submat(new Rect(
                new Point((x1 * input.cols()), (y1 * input.rows())),
                new Point((x2 * input.cols()), (y2 * input.rows()))));

        Core.split(submat, channels);

        double meanB = Core.mean(channels.get(0)).val[0];
        double meanG = Core.mean(channels.get(1)).val[0];
        double meanR = Core.mean(channels.get(2)).val[0];
        double max = Math.max(meanB, Math.max(meanG, meanR));

        if(max == meanB) parkingZone = 3;
        else if(max == meanG) parkingZone = 2;
        else parkingZone = 1;

        Imgproc.cvtColor(submat, output, Imgproc.COLOR_BGR2RGBA);
        return output;
    }

    public int getZone() {
        return parkingZone;
    }
}