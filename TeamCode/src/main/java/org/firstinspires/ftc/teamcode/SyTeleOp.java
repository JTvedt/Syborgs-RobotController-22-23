package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Jeffrey TeleOp")
public class SyTeleOp extends LinearOpMode {
    public RobotMethods robot;

    public boolean a, x;
    public boolean uPad, dPad, lPad, rPad;

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
        double drive = gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x * 0.6;

        double stickAngle = Math.atan2(drive, strafe);

        // If pressing B only move in four simple directions
        if (gamepad1.b) stickAngle = Math.round(stickAngle * 2 / Math.PI) * (Math.PI / 2);

        double angle = stickAngle - robot.getAngle();
        double magnitude = Math.hypot(drive, strafe);
        double multiplier = (gamepad1.y ? 1.4 : 1) * (rTrigger(1) ? 0.25 : 1);

         robot.teleDrive(stickAngle, magnitude * multiplier, turn * multiplier);
        // robot.teleDrive(drive * magnitude, strafe * magnitude, turn, multiplier);

        // Crane subsystem
        if (gamepad1.dpad_up && !uPad) robot.setSlides(-4300);
        if (gamepad1.dpad_right && !rPad) robot.setSlides(-2000);
        if (gamepad1.dpad_down && !dPad) robot.setSlides(0);

        // Locks slide position
        if (gamepad1.dpad_left && !lPad) robot.setSlides(robot.slidePosition());

        // Manual movement by player
        if (gamepad1.x && !x) robot.setSlides(robot.slideTarget() + (rTrigger(1) ? 35 : 100) * (lTrigger(1) ? -1 : 1));
        if (gamepad2.right_stick_y != 0) robot.moveSlides(gamepad2.right_stick_y);

        // Claw subsystem
        if (gamepad1.a && !a) robot.toggleClaw();

        // Reset buttons
        a = gamepad1.a;
        x = gamepad1.x;
        uPad = gamepad1.dpad_up;
        dPad = gamepad1.dpad_down;
        lPad = gamepad1.dpad_left;
        rPad = gamepad1.dpad_right;

        telemetry.addData("Robot angle", robot.getAngle());

        // telemetry.addData("Drive", drive);
        // telemetry.addData("Strafe", strafe);
        // telemetry.addData("Turn", turn);

        telemetry.addData("Stick Angle", stickAngle);
        telemetry.addData("Angle", angle);
        telemetry.addData("Magnitude", magnitude);

        // telemetry.addData("Slide target", robot.slideTarget());
        // telemetry.addData("Slide position", robot.slidePosition());
        // telemetry.addData("Claw state", robot.pinch ? "closed" : "open");

        telemetry.update();
    }

    public boolean lTrigger(int player) {
        if (player == 1) return gamepad1.left_trigger > .5;
        else return gamepad2.left_trigger > .5;
    }

    public boolean rTrigger(int player) {
        if (player == 1) return gamepad1.right_trigger > .5;
        else return gamepad2.right_trigger > .5;
    }
}