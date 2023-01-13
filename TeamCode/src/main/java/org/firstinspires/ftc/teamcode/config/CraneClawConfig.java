package org.firstinspires.ftc.teamcode.config;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

// This TeleOp is for testing purposes only (motors, etc)
@TeleOp(name="Crane & Claw Subsystem Testing")
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
            leftSlide.setPower(-0.5);
            rightSlide.setPower(-0.5);
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