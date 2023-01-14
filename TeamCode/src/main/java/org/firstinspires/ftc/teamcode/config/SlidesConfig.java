package org.firstinspires.ftc.teamcode.config;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp(name="Slide Motors Config", group="config")
public class SlidesConfig extends LinearOpMode {
    DcMotor leftSlide, rightSlide;

    @Override
    public void runOpMode() {
        leftSlide = hardwareMap.get(DcMotor.class, "LS");
        rightSlide = hardwareMap.get(DcMotor.class, "RS");

        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        while (opModeIsActive()) runLoop();
    }

    public void runLoop() {
        leftSlide.setPower(gamepad1.left_stick_y * 0.25);
        rightSlide.setPower(gamepad1.right_stick_y * 0.25);
    }
}
