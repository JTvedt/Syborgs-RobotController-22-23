package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Jeffrey TeleOp")
public class SyTeleOp extends LinearOpMode {
    public RobotMethods robot;

    // Controls claw opening
    public boolean a;

    @Override
    public void runOpMode() {
        robot = new RobotMethods(this);
        while (opModeIsActive()) runLoop();
    }

    // Runs constantly
    public void runLoop() {
        // Invert lStick Y so forward is positive
        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        double stickAngle = Math.atan2(drive, strafe);
        // If pressing B only move in four simple directions
        if (gamepad1.b) stickAngle = Math.round(stickAngle * 2 / Math.PI) * (Math.PI / 2);

        double angle = stickAngle - robot.getAngle();
        double magnitude = Math.hypot(drive, strafe) * 0.5;

        // Buttons to modify magnitude
        if (gamepad1.y) magnitude *= 2;
        if (gamepad1.left_trigger > 0.5) magnitude /= 3;

        robot.teleDrive(angle, magnitude, turn);

        // Crane subsystem
        if (gamepad1.dpad_up) robot.setSlides(-2840);
        if (gamepad1.dpad_down) robot.setSlides(0);

        // Claw subsystem
        if (gamepad1.a && !a) robot.toggleClaw();
        a = gamepad1.a;

        telemetry.addData("Angle:", stickAngle);
        telemetry.addData("Magnitude:", magnitude);
        telemetry.addData("Robot angle:", robot.getAngle());
        telemetry.addData("Slide position:", robot.leftSlide.getTargetPosition());
        telemetry.update();
    }
}