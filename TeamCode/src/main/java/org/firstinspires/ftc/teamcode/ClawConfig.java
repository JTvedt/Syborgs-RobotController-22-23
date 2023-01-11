package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Claw Config")
public class ClawConfig extends OpMode {
    private DcMotor leftSlide;
    private DcMotor rightSlide;

    private Servo leftClaw;
    private Servo rightClaw;

    private double leftPos = 0.1;
    private double rightPos = 0.9;

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
        double up = gamepad2.left_stick_y * 0.5;
        leftSlide.setPower(up);
        rightSlide.setPower(up);

        double multiplier = (gamepad2.left_trigger > 0.5 ? 4d : 1d) / (gamepad2.right_trigger > 0.5 ? 2d : 1d);

        leftPos += multiplier * ((gamepad2.a && !a ? 0.05 : 0) - (gamepad2.b && !b ? 0.05 : 0));
        rightPos += multiplier * ((gamepad2.x && !x ? 0.05 : 0) - (gamepad2.y && !y ? 0.05 : 0));

        leftClaw.setPosition(leftPos);
        rightClaw.setPosition(rightPos);

        telemetry.addData("left claw pos", "%.2f", leftPos);
        telemetry.addData("right claw pos", "%.2f", rightPos);
        telemetry.update();

        a = gamepad2.a;
        b = gamepad2.b;
        x = gamepad2.x;
        y = gamepad2.y;
    }
}
