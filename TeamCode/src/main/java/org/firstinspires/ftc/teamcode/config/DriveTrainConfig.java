package org.firstinspires.ftc.teamcode.config;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Tests the motors on the hardware map to test mapping.
 * X and Y to move the motors labelled Fl and FR.
 * A and B to move the motors labelled BL and BR.
 * Left stick to move all wheels forward or backwards.
 */
@TeleOp(name="Drive Train Config", group="config")
public class DriveTrainConfig extends OpMode {
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;

    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotor.class, "FL");
        frontRight = hardwareMap.get(DcMotor.class, "FR");
        backLeft = hardwareMap.get(DcMotor.class, "BL");
        backRight = hardwareMap.get(DcMotor.class, "BR");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        frontLeft.setPower(0.0);
        frontRight.setPower(0.0);
        backLeft.setPower(0.0);
        backRight.setPower(0.0);

        if (gamepad1.y) frontLeft.setPower(0.35);
        if (gamepad1.b) frontRight.setPower(0.35);
        if (gamepad1.x) backLeft.setPower(0.35);
        if (gamepad1.a) backRight.setPower(0.35);

        if (gamepad1.left_stick_y != 0) {
            frontLeft.setPower(-gamepad1.left_stick_y * 0.5);
            frontRight.setPower(-gamepad1.left_stick_y * 0.5);
            backLeft.setPower(-gamepad1.left_stick_y * 0.5);
            backRight.setPower(-gamepad1.left_stick_y * 0.5);
        }
    }
}
