package org.firstinspires.ftc.teamcode.configteleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Tests the claw to determine the grip strength necessary to carry objects.
 * X and A to close the left and right claw.
 * Y and B to open the left and right claw.
 * LT to increase claw increments.
 * RT to decrease claw increments.
 * Left stick to move the slides up and down.
 * Claw positions are shown on telemetry.
 */
@TeleOp(name="Claw Config", group="config")
public class ClawConfig extends OpMode {
    private DcMotor leftSlide;
    private DcMotor rightSlide;

    private Servo leftClaw;
    private Servo rightClaw;

    private double leftPos = 1.0;
    private double rightPos = 0.0;

    private boolean a = false, b = false;
    private boolean x = false, y = false;

    @Override
    public void init() {
        leftSlide = hardwareMap.get(DcMotor.class, "LS");
        rightSlide = hardwareMap.get(DcMotor.class, "RS");

        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setDirection(DcMotor.Direction.REVERSE);

        leftClaw = hardwareMap.get(Servo.class, "LC");
        rightClaw = hardwareMap.get(Servo.class, "RC");
    }

    @Override
    public void loop() {
        double up = gamepad1.left_stick_y * 0.7;
        leftSlide.setPower(up);
        rightSlide.setPower(up);

        double multiplier = (gamepad1.left_trigger > 0.5 ? 4 : 1) * (gamepad1.right_trigger > 0.5 ? 0.2 : 1);

        leftPos += multiplier * ((gamepad1.y && !y ? 0.05 : 0) - (gamepad1.x && !x ? 0.05 : 0));
        rightPos += multiplier * ((gamepad1.a && !a ? 0.05 : 0) - (gamepad1.b && !b ? 0.05 : 0));

        leftClaw.setPosition(leftPos);
        rightClaw.setPosition(rightPos);

        telemetry.addData("left claw pos", "%.2f", leftPos);
        telemetry.addData("right claw pos", "%.2f", rightPos);
        telemetry.update();

        a = gamepad1.a;
        b = gamepad1.b;
        x = gamepad1.x;
        y = gamepad1.y;
    }
}
