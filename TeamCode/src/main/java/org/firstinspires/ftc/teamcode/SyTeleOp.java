package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.SyBot;

@TeleOp(name="Jeffrey TeleOp")
public class SyTeleOp extends LinearOpMode {
    public SyBot robot;

    public boolean a, b, x, y;
    public boolean uPad, dPad, lPad, rPad;

    @Override
    public void runOpMode() {
        robot = new SyBot(this, SyBot.OpModeType.TELEOP);
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
        double turn = gamepad1.right_stick_x * 0.75;

        double stickAngle = Math.atan2(drive, strafe);
        double magnitude = Math.hypot(drive, strafe);
        double multiplier = 0.6 * (gamepad1.b ? 1.4 : 1) * (rTrigger(1) ? 0.35 : 1);

        // Precision input w/ D-Pad
        if (gamepad1.dpad_up) {
            stickAngle = Math.PI/2;
            magnitude = 0.35;
        } else if (gamepad1.dpad_down) {
            stickAngle = 3 * Math.PI/2;
            magnitude = 0.35;
        } else if (gamepad1.dpad_left) {
            stickAngle = Math.PI;
            magnitude = 0.35;
        } else if (gamepad1.dpad_right) {
            stickAngle = 0;
            magnitude = 0.35;
        }

        // Turn control
        if (gamepad1.left_bumper) robot.resetAngle();
        if (lTrigger(1)) {
            double roundedAngle = Math.round(robot.getAngle() * 2/Math.PI) * Math.PI/2;
            if (robot.getAngle() != roundedAngle) turn = 0.3 * (robot.getAngle() < roundedAngle ? -1 : 1);
        }

        // Plug in numbers
        robot.teleDrive(stickAngle, magnitude, turn, multiplier);

        // P2 Crane subsystem
        if (gamepad2.dpad_up && !uPad) robot.setSlides(-4300);
        if (gamepad2.dpad_right && !rPad) robot.setSlides(-2000);
        if (gamepad2.dpad_down && !dPad) robot.setSlides(0);
        if (gamepad2.dpad_left && !lPad) robot.setSlides(robot.slidePosition()); // Locks slides position

        // Manual slide fine tuning
        if (gamepad2.x && !x) robot.setSlides(robot.slideTarget() + (rTrigger(2) ? 35 : 100));
        if (gamepad2.y && !y) robot.setSlides(robot.slideTarget() - (rTrigger(2) ? 35 : 100));
        if (gamepad2.right_stick_y != 0) robot.manualSlides = true;
        if (robot.manualSlides) robot.moveSlides(gamepad2.right_stick_y * 0.5);
        if (gamepad2.left_trigger > .5) robot.moveSlides(1.0);
        if (gamepad2.left_bumper) robot.resetSlides();

        // Claw subsystem
        if (gamepad2.a && !a) robot.toggleClaw(); // Regulr cones
        if (gamepad2.b && !b) robot.setClaw(0.25); // Capstone

        // Reset buttons
        a = gamepad2.a;
        b = gamepad2.b;
        y = gamepad2.y;
        x = gamepad2.x;
        uPad = gamepad2.dpad_up;
        dPad = gamepad2.dpad_down;
        lPad = gamepad2.dpad_left;
        rPad = gamepad2.dpad_right;

        telemetry.addData("Robot angle", robot.getAngle());

        // telemetry.addData("Drive", drive);
        // telemetry.addData("Strafe", strafe);
        // telemetry.addData("Turn", turn);

        // telemetry.addData("Stick Angle", stickAngle);
        // telemetry.addData("Angle", stickAngle - robot.getAngle());
        // telemetry.addData("Magnitude", magnitude);

        // telemetry.addData("Slide target", robot.slideTarget());
        // telemetry.addData("Slide position", robot.slidePosition());
        // telemetry.addData("LSlide power", robot.leftSlide.getPower());
        // telemetry.addData("RSlide power", robot.rightSlide.getPower());

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