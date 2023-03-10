package org.firstinspires.ftc.teamcode.deprecated;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.Sybot;

/**
 * Places one cone on the high junction and parks. Starting position on the left side
 * @author Jeffrey Tvedt
 */
@Deprecated
@Disabled
@Autonomous(name="1C Left Autonomous AT")
public class SingleLeftAT extends LinearOpMode {
    public Sybot robot;

    @Override
    public void runOpMode() {
        Sybot.setImplementation(Sybot.CvImplementation.APRIL_TAGS);
        robot = new Sybot(this, Sybot.OpModeType.AUTONOMOUS, Sybot.StartSide.LEFT);
        robot.mirrorDirection = true;

        robot.setClaw(true);
        int parkZone = robot.retrieveZone();

        // Autonomous processes go here
        // Everything in this section is mirrored
        robot.setSlides(-4130);
        robot.drive(56);
        robot.strafe(-13);
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
        robot.mirrorDirection = false;
        if (parkingSpot == 9) robot.strafe(-26);
        else if (parkingSpot == 11) robot.strafe(26);
    }
}