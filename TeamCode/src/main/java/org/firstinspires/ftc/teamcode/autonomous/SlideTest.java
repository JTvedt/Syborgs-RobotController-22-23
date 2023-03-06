package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.util.Sybot;

@Autonomous(name="Slide Test", group="test")
public class SlideTest extends LinearOpMode {
    public Sybot robot;

    @Override
    public void runOpMode() {
        robot = new Sybot(this, Sybot.OpModeType.AUTONOMOUS, Sybot.StartSide.LEFT);

        robot.setSlides(-400);
        robot.waitForSlides();
        robot.setSlides(0);
        robot.waitForSlides();
    }
}