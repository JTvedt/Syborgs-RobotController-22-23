package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Jeffrey TeleOp")
public class SyTeleOp extends LinearOpMode {
    public RobotMethods robot;

    // Controls claw opening
    public boolean a, x;

    @Override
    public void runOpMode() {
        robot = new RobotMethods(this);
        robot.toggleClaw(false);

        while (opModeIsActive()) {
            runLoop();
        }
    }

    // Runs constantly
    public void runLoop() {
        // Invert lStick Y so forward is positive
        double drive = -gamepad1.left_stick_y * 0.7;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x * 0.6;

        double stickAngle = Math.atan2(drive, strafe);
        // If pressing B only move in four simple directions
        if (gamepad1.b) stickAngle = Math.round(stickAngle * 2 / Math.PI) * (Math.PI / 2);

        double angle = stickAngle - robot.getAngle();
        angle = Math.PI * 3/4;
        double magnitude = Math.hypot(drive, strafe);
        double multiplier = 1;

        // Buttons to modify magnitude
        if (gamepad1.y) multiplier *= 1.4;
        if (gamepad1.right_trigger > 0.5) multiplier /= 4;

        // robot.teleDrive(angle, magnitude * multiplier, turn);
        robot.teleDrive(drive * magnitude, strafe * magnitude, turn, multiplier);

        // Crane subsystem
        if (gamepad1.dpad_right) robot.setSlides(-2000);
        if (gamepad1.dpad_up) robot.setSlides(-4300);
        if (gamepad1.dpad_down) robot.setSlides(0);

        //TODO make this cleaner
        if (gamepad1.x && !x && gamepad1.left_trigger < .5) robot.setSlides(robot.leftSlide.getTargetPosition() - 100);
        if (gamepad1.x && !x && gamepad1.left_trigger > .5) robot.setSlides(robot.leftSlide.getTargetPosition() +  (gamepad1.right_trigger > .5 ? 35 : 100));

        // Claw subsystem
        if (gamepad1.a && !a) robot.toggleClaw();

        // Reset buttons
        a = gamepad1.a;
        x = gamepad1.x;

        telemetry.addData("Drive", drive);
        telemetry.addData("Strafe", strafe);
        telemetry.addData("Turn", turn);
        telemetry.addData("Stick Angle", stickAngle);
        telemetry.addData("Angle", angle);
        telemetry.addData("Magnitude", magnitude);
        telemetry.addData("Robot angle", robot.getAngle());
        telemetry.addData("Slide target", robot.leftSlide.getTargetPosition());
        telemetry.addData("Slide position", robot.leftSlide.getCurrentPosition());
        telemetry.addData("Claw state", robot.pinch ? "closed" : "open");

        telemetry.update();
    }
}
