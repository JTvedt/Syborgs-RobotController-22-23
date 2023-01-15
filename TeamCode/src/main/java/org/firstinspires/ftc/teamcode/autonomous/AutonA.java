package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.SyBot;

@Autonomous(name="Test Autonomous A", group="test")
public class AutonA extends LinearOpMode {
    public SyBot robot;

    @Override
    public void runOpMode() {
        robot = new SyBot(this);
        robot.toggleClaw(true);

        // Autonomous processes go here
        robot.setSlides(-2000);
        robot.waitForSlides();
        robot.setSlides(-1600);
        robot.waitForSlides();
        robot.toggleClaw();
        sleep(300);
        robot.setSlides(0);
        robot.waitForSlides();
    }
}