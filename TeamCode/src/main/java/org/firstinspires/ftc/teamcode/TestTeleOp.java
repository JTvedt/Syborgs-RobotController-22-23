package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

// This TeleOp is for testing purposes only (motors, etc)
@TeleOp(name="Test TeleOp")
public class TestTeleOp extends OpMode {
    private DcMotor leftSlide;
    private DcMotor rightSlide;

    @Override
    public void init() {
        leftSlide = hardwareMap.get(DcMotor.class, "LS");
        rightSlide = hardwareMap.get(DcMotor.class, "RS");
    }

    @Override
    public void loop() {
        leftSlide.setPower(0.0);
        rightSlide.setPower(0.0);

        if (gamepad1.a) {
            leftSlide.setPower(0.4);
            rightSlide.setPower(0.4);
        }
        if (gamepad1.b) {
            leftSlide.setPower(-0.4);
            rightSlide.setPower(-0.4);
        }
        if (gamepad1.x) {
            leftSlide.setPower(0.85);
            rightSlide.setPower(0.85);
        }
        if (gamepad1.y) {
            leftSlide.setPower(-0.85);
            rightSlide.setPower(-0.85);
        }
    }
}
