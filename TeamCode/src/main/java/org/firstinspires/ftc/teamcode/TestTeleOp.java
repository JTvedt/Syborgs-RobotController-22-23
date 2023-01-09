package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

// This TeleOp is for testing purposes only (motors, etc)
@TeleOp(name="Test TeleOp")
public class TestTeleOp extends OpMode {
    private DcMotor crane;

    @Override
    public void init() {
        crane = hardwareMap.get(DcMotor.class, "Crane");
    }

    @Override
    public void loop() {
        if (gamepad1.a) crane.setPower(0.4);
        if (gamepad1.b) crane.setPower(-0.4);
        if (gamepad1.x) crane.setPower(0.85);
        if (gamepad1.y) crane.setPower(-0.85);
    }
}
