package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.SyBot;
import org.firstinspires.ftc.teamcode.cv.CvPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="Test Autonomous B", group="test")
public class AutonB extends LinearOpMode {
    public SyBot robot;
    public CvPipeline pipeline;
    public OpenCvCamera camera;

    @Override
    public void runOpMode() {
        //Initialize CV + camera
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Camera");
        pipeline = new CvPipeline();
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

        robot = new SyBot(this);

        // Autonomous processes go here
        // TODO test the CvPipeline
        sleep(1000);
        int parkZone = pipeline.getZone();
        telemetry.addData("Parking Zone", parkZone == -1 ? "..." : parkZone);
        telemetry.update();
        sleep(1000);
    }
}