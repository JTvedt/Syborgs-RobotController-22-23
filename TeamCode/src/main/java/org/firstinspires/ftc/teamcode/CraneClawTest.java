package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

// This TeleOp is for testing purposes only (motors, etc)
@Disabled
@TeleOp(name="Crane & Claw Subsystm Testing")
public class CraneClawTest extends OpMode {
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

        if (gamepad2.a) {
            leftSlide.setPower(0.5);
            rightSlide.setPower(0.5);
        }
        if (gamepad2.b) {
            leftSlide.setPower(-0.5);
            rightSlide.setPower(-0.5);
        }

        if (gamepad2.x) {
            leftClaw.setPosition(0.1);
            rightClaw.setPosition(0.9);
        }
        if (gamepad2.y) {
            leftClaw.setPosition(0.9);
            rightClaw.setPosition(0.1);
        }
    }
}
