package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Jeffrey TeleOp")
public class SyTeleOp extends LinearOpMode {
    public RobotMethods robot;

    public boolean a, y, x, a2;
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
        // P1 Stick Input
        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x * 0.6;

        double stickAngle = Math.atan2(drive, strafe);
        double magnitude = Math.hypot(drive, strafe);
        double multiplier = 0.6 * (gamepad1.b ? 1.4 : 1) * (rTrigger(1) ? 0.35 : 1);

        // P2 Precision input
        if (gamepad2.dpad_up) {
            stickAngle = Math.PI/2;
            magnitude = 0.35;
        } else if (gamepad2.dpad_down) {
            stickAngle = 3 * Math.PI/2;
            magnitude = 0.35;
        } else if (gamepad2.dpad_left) {
            stickAngle = Math.PI;
            magnitude = 0.35;
        } else if (gamepad2.dpad_right) {
            stickAngle = 0;
            magnitude = 0.35;
        }

        // Plug in numbers
        robot.teleDrive(stickAngle, magnitude, turn, multiplier);

        // Crane subsystem
        if (gamepad1.dpad_up && !uPad) robot.setSlides(-4300);
        if (gamepad1.dpad_right && !rPad) robot.setSlides(-2000);
        if (gamepad1.dpad_down && !dPad) robot.setSlides(0);
        if (gamepad1.dpad_left && !lPad) robot.setSlides(robot.slidePosition()); // Locks slides position

        // Manual movement by player
        if (gamepad1.x && !x) robot.setSlides(robot.slideTarget() + (rTrigger(1) ? 35 : 100));
        if (gamepad1.y && !y) robot.setSlides(robot.slideTarget() - (rTrigger(1) ? 35 : 100));
        if (gamepad2.right_stick_y != 0) robot.manualSlides = true;
        if (robot.manualSlides) robot.moveSlides(gamepad2.right_stick_y * 0.5);
        if (gamepad1.left_trigger > .5) robot.moveSlides(0.5);

        // Claw subsystem
        if (gamepad1.a && !a) robot.toggleClaw();
        if (gamepad2.a && !a2) robot.setClaw(0.15); // Capstone claw

        // Reset buttons
        a = gamepad1.a;
        y = gamepad1.y;
        x = gamepad1.x;
        a2 = gamepad2.a;
        uPad = gamepad1.dpad_up;
        dPad = gamepad1.dpad_down;
        lPad = gamepad1.dpad_left;
        rPad = gamepad1.dpad_right;

        telemetry.addData("Robot angle", robot.getAngle());

        // telemetry.addData("Drive", drive);
        // telemetry.addData("Strafe", strafe);
        // telemetry.addData("Turn", turn);

        // telemetry.addData("Stick Angle", stickAngle);
        // telemetry.addData("Angle", stickAngle - robot.getAngle());
        // telemetry.addData("Magnitude", magnitude);

        telemetry.addData("Slide target", robot.slideTarget());
        telemetry.addData("Slide position", robot.slidePosition());
        telemetry.addData("LSlide power", robot.leftSlide.getPower());
        telemetry.addData("RSlide power", robot.rightSlide.getPower());

        telemetry.addData("Claw state", robot.pinch ? "closed" : "open");

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