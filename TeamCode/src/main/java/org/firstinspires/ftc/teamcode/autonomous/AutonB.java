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

    @Override
    public void runOpMode() {
        robot = new Sybot(this, Sybot.OpModeType.AUTONOMOUS, Sybot.StartSide.LEFT);

        robot.drive(24);
        robot.spin(45);
        robot.polarMove(12, 45);
    }
}