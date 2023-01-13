package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp(name="Configure Slides")
public class SlidesTest extends LinearOpMode {
    DcMotor leftSlide, rightSlide;
    int slideHeight;
    boolean a, b;

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
        leftSlide.setPower(gamepad1.left_stick_y);
        rightSlide.setPower(gamepad1.left_stick_y);
    }
}
