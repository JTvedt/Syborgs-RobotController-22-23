package org.firstinspires.ftc.teamcode.config;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.util.Sybot;

/**
 * Tests the encoder values of the slides to determine certain heights.
 * No drive train functionality, push the robot manually.
 * X and Y to inch the slides down and up.
 * A to toggle the claw between open and close.
 * Slide encoder position shown on telemetry.
 */
@TeleOp(name="Slides Encoder Config", group="config")
public class SlidesEncodersConfig extends LinearOpMode {
    public Sybot robot;
    int slidesHeight = 0;
    boolean x, y;
    boolean a;

    @Override
    public void runOpMode() {
        robot = new Sybot(this);

        robot.leftSlide.setTargetPosition(0);
        robot.rightSlide.setTargetPosition(0);

        robot.leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.leftSlide.setPower(0.7);
        robot.rightSlide.setPower(0.7);

        while (opModeIsActive()) runLoop();
    }

    public void runLoop() {
        robot.leftSlide.setTargetPosition(slidesHeight);
        robot.rightSlide.setTargetPosition(slidesHeight);

        if (gamepad1.a && !a) robot.toggleClaw();
        if (gamepad1.x && !x) slidesHeight += gamepad1.right_trigger > .5 ? 50 : 250;
        if (gamepad1.y && !y) slidesHeight -= gamepad1.right_trigger > .5 ? 50 : 250;

        a = gamepad1.a;
        x = gamepad1.x;
        y = gamepad1.y;

        telemetry.addData("Slide Height", slidesHeight);
        telemetry.addData("Left Slide", robot.leftSlide.getCurrentPosition());
        telemetry.addData("Right Slide", robot.rightSlide.getCurrentPosition());
        telemetry.update();
    }
}
