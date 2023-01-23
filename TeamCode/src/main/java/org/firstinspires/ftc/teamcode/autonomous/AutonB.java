package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.Sybot;
import org.firstinspires.ftc.teamcode.cv.EasyOpenCvPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="Test Autonomous B", group="test")
public class AutonB extends LinearOpMode {
    public Sybot robot;
    public EasyOpenCvPipeline pipeline;
    public OpenCvCamera camera;

    @Override
    public void runOpMode() {
        //Initialize CV + camera
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Camera");
        pipeline = new EasyOpenCvPipeline();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        camera.setPipeline(pipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        robot = new Sybot(this);

        // Autonomous processes go here
        // TODO test the CvPipeline
        sleep(1000);
        int parkZone = pipeline.getZone();
        telemetry.addData("Parking Zone", parkZone == -1 ? "..." : parkZone);
        telemetry.update();
        sleep(1000);
    }
}