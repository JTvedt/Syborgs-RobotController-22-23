package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.Sybot;

@Autonomous(name="Regionals Auton (R)")
public class RegAutonRight extends LinearOpMode {
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
        robot.drive(2.4);

        placeCone();
        getCone(-140);
        placeCone();
        getCone(-120);
        placeCone();

        robot.spinTo(0);
        robot.setSlides(0);
        robot.drive(-2);

//        park();
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
        robot.cartesianMove(.31, -.31);
        robot.setSlides(0);
    }

    void getCone(int height) {
        // Move to stack
        robot.spinTo(-90);
        robot.setSlides(height);
        robot.strafe(1.38);

        // Pick from stack
        robot.waitForSlides();
        robot.setClaw(true);
        sleep(400);
        robot.setSlides(Sybot.SLIDE_HIGH_TICKS);
        sleep(200);

        // Return to tile
        robot.strafe(-1.38);
    }

    void park() {

    }
}
