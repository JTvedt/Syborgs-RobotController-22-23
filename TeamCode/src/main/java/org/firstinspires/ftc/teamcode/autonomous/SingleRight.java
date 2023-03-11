package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.Sybot;

@Autonomous(name="1C Auton (R)")
public class SingleRight extends LinearOpMode {
    public Sybot robot;

    @Override
    public void runOpMode() {
        Sybot.setImplementation(Sybot.CvImplementation.APRIL_TAGS);
        robot = new Sybot(this, Sybot.OpModeType.AUTONOMOUS);
        robot.setDriveUnit(DistanceUnit.TILES);
        robot.retrieveZone();
        robot.setClaw(true);
        sleep(200);
        robot.setSlides(Sybot.SLIDE_HIGH_TICKS);

        // Move to first junction
        robot.drive(2.3);

        placeCone();

        robot.spinTo(0);
        robot.setSlides(0);
        robot.drive(-1);

        park();
        robot.waitForSlides();
    }

    void placeCone() {
        // Move to junction
        robot.spinTo(45);
        robot.cartesianMove(-.31, .31);
        robot.waitForSlides();

        // Place cone
        robot.setSlides(-710);
        sleep(400);
        robot.setClaw(false);

        // Retreat
        robot.cartesianMove(.31, -.33);
        robot.setSlides(0);
    }

    void park() {
        if (robot.parkZone == 9)
            robot.strafe(-1);
        else if (robot.parkZone == 11)
            robot.strafe(1);
    }
}
