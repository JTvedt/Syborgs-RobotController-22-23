package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RobotMethods;
import org.firstinspires.ftc.teamcode.cv.CvPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="1C Left Autonomous")
public class SingleLeft extends LinearOpMode {
    public RobotMethods robot;
    public CvPipeline pipeline;
    public OpenCvCamera camera;

    @Override
    public void runOpMode() {
        robot = new RobotMethods(this, RobotMethods.OpModeType.AUTONOMOUS);

        robot.toggleClaw(true);
        sleep(400);
        int parkZone = robot.pipeline.getZone();
        telemetry.addData("Parking in", parkZone);
        telemetry.update();
        sleep(600);

        // Autonomous processes go here
        robot.setSlides(-4130);
        robot.drive(56);
        robot.strafe(12);
        robot.waitForSlides();
        robot.drive(3);
        robot.setSlides(0);
        sleep(300);
        robot.toggleClaw();
        sleep(300);
        robot.drive(-3);
        robot.strafe(-12);
        robot.drive(-26);

        park(parkZone);
        robot.waitForSlides();
        sleep(800);
    }

    // 0 for red, 1 for green, 2 for blue
    public void park(int parkingSpot) {
        if (parkingSpot == 0) robot.strafe(-26);
        else if (parkingSpot == 2) robot.strafe(26);
    }
}