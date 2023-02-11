package org.firstinspires.ftc.teamcode.deprecated;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.Sybot;
import org.firstinspires.ftc.teamcode.cv.EasyOpenCvPipeline;
@Disabled
@Deprecated
@Autonomous(name="2C Right Autonomous")
public class DoubleRight extends LinearOpMode {
    Sybot robot;

    @Override
    public void runOpMode() {
        Sybot.setImplementation(Sybot.CvImplementation.APRIL_TAGS);
        robot = new Sybot(this, Sybot.OpModeType.AUTONOMOUS, Sybot.StartSide.RIGHT);

        // Initial Setup
        robot.setClaw(true);
        robot.retrieveZone();

        // Autonomous processes go here
        // Position for cone 1
        robot.setSlides(Sybot.SLIDE_HIGH_TICKS);
        robot.drive(54);
        robot.spin(45);
        robot.waitForSlides();
        placeCone();

        // Move to park
        robot.spin(135);
        robot.drive(-24);
        park();

        robot.waitForSlides();
        sleep(4000);
        robot.stop();
    }

    public void placeCone() {
        robot.cartesianMove(-8, 8);      // Move to junction
        robot.setSlides(-4000);         // Lower slides slightly
        sleep(600);
        robot.toggleClaw();             // Open claw to drop cone
        robot.setSlides(Sybot.SLIDE_HIGH_TICKS); // Raise slides back up
        robot.cartesianMove(8, -8);    // Move away from junction
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