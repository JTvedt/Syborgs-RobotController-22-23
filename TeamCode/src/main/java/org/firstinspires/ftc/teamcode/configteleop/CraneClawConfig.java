package org.firstinspires.ftc.teamcode.configteleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Tests the crane and claw subsystem.
 * X and Y to close and open the claw.
 * B and A to move the slides up and down.
 */
@TeleOp(name="Crane & Claw Config", group="config")
public class CraneClawConfig extends OpMode {
    private DcMotor leftSlide;
    private DcMotor rightSlide;

    private Servo leftClaw;
    private Servo rightClaw;

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
        leftSlide.setPower(0.0);
        rightSlide.setPower(0.0);

        if (gamepad1.a) {
            leftSlide.setPower(0.5);
            rightSlide.setPower(0.5);
        }
        if (gamepad1.b) {
            leftSlide.setPower(-0.85);
            rightSlide.setPower(-0.85);
        }

        if (gamepad1.x) { // Close
            leftClaw.setPosition(0.7);
            rightClaw.setPosition(0.3);
        }
        if (gamepad1.y) { // Open
            leftClaw.setPosition(1.0);
            rightClaw.setPosition(0.0);
        }
    }
}