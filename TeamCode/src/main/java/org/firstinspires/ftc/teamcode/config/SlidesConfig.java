package org.firstinspires.ftc.teamcode.config;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Tests each slide motor individually, useful for de-tangling slides.
 * Hold claw up with hand manually, and then move slides.
 * Move left and right stick to control left and right slide.
 * As the slides move up, the spool loosens, and it tightens on the way down.
 */
@TeleOp(name="Slide Motors Config", group="config")
public class SlidesConfig extends LinearOpMode {
    DcMotor leftSlide, rightSlide;

    @Override
    public void runOpMode() {
        leftSlide = hardwareMap.get(DcMotor.class, "LS");
        rightSlide = hardwareMap.get(DcMotor.class, "RS");

        rightSlide.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        while (opModeIsActive()) runLoop();
    }

    public void runLoop() {
        leftSlide.setPower(gamepad1.left_stick_y * 0.25);
        rightSlide.setPower(gamepad1.right_stick_y * 0.25);
    }
}
