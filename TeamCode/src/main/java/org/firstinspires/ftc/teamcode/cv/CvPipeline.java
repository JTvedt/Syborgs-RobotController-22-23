package org.firstinspires.ftc.teamcode.cv;

import org.opencv.core.Core;
import org.opencv.core.Mat;
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

    @Override
    public Mat processFrame(Mat input) {
        double cropSize = 0.24;
        double offsetX = 0.45;
        double offsetY = 0.08;

        int size = (int)Math.min(input.rows() * cropSize, input.cols() * cropSize);

        Imgproc.cvtColor(input, bgrInput, Imgproc.COLOR_RGBA2BGR);

        submat = bgrInput.submat(new Rect(
                (int)(offsetX * input.rows()),
                (int)(offsetY * input.cols()),
                size, size));

        Core.split(submat, channels);

        double meanB = Core.mean(channels.get(0)).val[0];
        double meanG = Core.mean(channels.get(1)).val[0];
        double meanR = Core.mean(channels.get(2)).val[0];
        double max = Math.max(meanB, Math.max(meanG, meanR));

        if(max == meanB) parkingZone = 2;
        else if(max == meanG) parkingZone = 1;
        else parkingZone = 0;

        Imgproc.cvtColor(submat, output, Imgproc.COLOR_BGR2RGBA);
        return output;
    }

    public int getZone() {
        return parkingZone;
    }
}