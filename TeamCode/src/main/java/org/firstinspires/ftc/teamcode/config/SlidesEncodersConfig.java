package org.firstinspires.ftc.teamcode.config;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.RobotMethods;

@TeleOp(name="Slides Encoder Config", group="config")
public class SlidesEncodersConfig extends LinearOpMode {
    public RobotMethods robot;
    int slidesHeight = 0;
    boolean x, y;

    @Override
    public void runOpMode() {
        robot = new RobotMethods(this);

        while (opModeIsActive()) runLoop();
    }

    public void runLoop() {
        robot.setSlides(slidesHeight);
        if (gamepad1.x && !x) slidesHeight += gamepad1.right_trigger > .5 ? 50 : 250;
        if (gamepad1.y && !y) slidesHeight -= gamepad1.right_trigger > .5 ? 50 : 250;

        x = gamepad1.x;
        y = gamepad1.y;

        telemetry.addData("Slide Height", slidesHeight);
        telemetry.addData("Slide Position", robot.slidePosition());
    }
}
