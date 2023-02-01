package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.Sybot;
import org.firstinspires.ftc.teamcode.cv.EasyOpenCvPipeline;

@Autonomous(name="2C Left Autonomous")
public class DoubleLeft extends LinearOpMode {
    Sybot robot;

    @Override
    public void runOpMode() {
        robot = new Sybot(this, Sybot.OpModeType.AUTONOMOUS);

        // Initial Setup
        robot.setClaw(true);
        robot.retrieveZone();

        // Autonomous processes go here
        // Position for cone 1
        robot.setSlides(Sybot.SLIDE_HIGH_TICKS);
        robot.drive(54);
        robot.spin(-45);
        robot.waitForSlides();
        placeCone();

        // Position for cone 2
        robot.spin(135);
        robot.setSlides(-730);
        robot.strafe(-28);
        takeCone();

        robot.dropSlides();
        robot.waitForSlides();
        sleep(10000);
    }

    public void placeCone() {
        robot.cartesianMove(8, 8);      // Move to junction
        robot.setSlides(-4000);         // Lower slides slightly
        sleep(600);
        robot.toggleClaw();             // Open claw to drop cone
        robot.setSlides(Sybot.SLIDE_HIGH_TICKS); // Raise slides back up
        robot.cartesianMove(-8, -8);    // Move away from junction
        robot.dropSlides();             // Drop slides
    }

    public void takeCone() {

    }

    // 1 for red, 2 for green, 3 for blue
    public void park() {
        if (robot.parkZone == 1) robot.strafe(-24);
        else if (robot.parkZone == 3) robot.strafe(24);
    }
}