package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.Sybot;
import org.firstinspires.ftc.teamcode.cv.EasyOpenCvPipeline;

@Autonomous(name="2C Left Autonomous")
public class DoubleLeft extends LinearOpMode {
    public Sybot robot;
    public EasyOpenCvPipeline cv;

    @Override
    public void runOpMode() {
        robot = new Sybot(this, Sybot.OpModeType.AUTONOMOUS);

        // Initial Setup
        robot.setClaw(true);
        sleep(200);
        int parkZone = robot.pipeline.getZone();
        telemetry.addData("Parking in", parkZone);
        telemetry.update();
        sleep(300);
        robot.camera.stopStreaming();

        // Autonomous processes go here

        // Position for cone 1
        robot.setSlides(Sybot.SLIDE_HIGH_TICKS);
        robot.drive(54);
        robot.spin(-45);
        robot.waitForSlides();
        robot.cartesianMove(8, 8);

        // Place Cone
        robot.setSlides(-4000);
        sleep(600);
        robot.toggleClaw();
        robot.setSlides(Sybot.SLIDE_HIGH_TICKS);
        robot.cartesianMove(-8, -8);
        robot.dropSlides();

        // Position for cone 2
        robot.spin(135);
        robot.setSlides(-730);
        robot.strafe(-28);

        robot.dropSlides();
        robot.waitForSlides();
        sleep(10000);
    }

    // 0 for red, 1 for green, 2 for blue
    public void park(int parkingSpot) {
        if (parkingSpot == 0) robot.strafe(-24);
        else if (parkingSpot == 1) robot.strafe(24);
    }
}