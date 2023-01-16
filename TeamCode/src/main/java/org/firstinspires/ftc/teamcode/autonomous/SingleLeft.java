package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.Sybot;
import org.firstinspires.ftc.teamcode.cv.CvPipeline;
import org.openftc.easyopencv.OpenCvCamera;

/**
 * Places one cone on the high junction and parks. Starting position on the left side
 * @author Jeffrey Tvedt
 */
@Autonomous(name="1C Left Autonomous")
public class SingleLeft extends LinearOpMode {
    public Sybot robot;
    public CvPipeline pipeline;
    public OpenCvCamera camera;

    @Override
    public void runOpMode() {
        robot = new Sybot(this, Sybot.OpModeType.AUTONOMOUS, Sybot.StartSide.LEFT);

        robot.setClaw(true);
        sleep(400);
        int parkZone = robot.pipeline.getZone();
        telemetry.addData("Parking in", parkZone);
        telemetry.update();
        sleep(600);
        robot.camera.stopStreaming();

        // Autonomous processes go here
        // Everything in this section is mirrored
        robot.setSlides(-4130);
        robot.drive(56);
        robot.strafe(-12);
        robot.waitForSlides();
        robot.drive(3);
        robot.setSlides(0);
        robot.rest();
        robot.toggleClaw();
        robot.rest();
        robot.drive(-3);
        robot.strafe(12);
        robot.drive(-26);

        park(parkZone);
        robot.waitForSlides();
        sleep(800);
    }

    // 0 for red, 1 for green, 2 for blue
    public void park(int parkingSpot) {
        if (parkingSpot == 0) robot.strafe(26);
        else if (parkingSpot == 2) robot.strafe(-26);
    }
}