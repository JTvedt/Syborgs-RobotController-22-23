package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.Sybot;

@Autonomous(name="Test Autonomous A", group="test")
public class AutonA extends LinearOpMode {
    public Sybot robot;

    @Override
    public void runOpMode() {
        robot = new Sybot(this);

        // Autonomous processes go here
        robot.setSlides(Sybot.SLIDE_HIGH_TICKS);
        robot.waitForSlides();
        sleep(400);
        robot.dropSlides();
        robot.waitForSlides();
    }
}