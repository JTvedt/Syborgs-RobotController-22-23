package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.Sybot;

@Autonomous(name="Reg Auton (R)")
public class RegAuton extends LinearOpMode {
    public Sybot robot;

    @Override
    public void runOpMode() {
        Sybot.setImplementation(Sybot.CvImplementation.APRIL_TAGS);
        robot = new Sybot(this, Sybot.OpModeType.AUTONOMOUS);
        robot.setDriveUnit(DistanceUnit.TILES);
        robot.setClaw(true);
        sleep(200);
        robot.setSlides(Sybot.SLIDE_HIGH_TICKS);

        // Move to first junction
        robot.drive(2.3);
        robot.spinTo(45);
        robot.cartesianMove(-.25, .25);
        robot.waitForSlides();

        // Place and retreat
        robot.setSlides(-760);
        sleep(400);
        robot.setClaw(false);
        robot.setSlides(Sybot.SLIDE_HIGH_TICKS);
        robot.cartesianMove(.25, -.25);
        robot.setSlides(0);

        // Move to stack
        robot.spinTo(-90);
        robot.strafe(1.2);
        robot.setSlides(-160);

        // Pick from stack
        robot.setClaw(true);
    }
}
