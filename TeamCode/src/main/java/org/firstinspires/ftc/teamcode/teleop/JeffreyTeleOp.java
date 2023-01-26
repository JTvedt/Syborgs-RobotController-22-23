package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.Sybot;
import org.firstinspires.ftc.teamcode.util.Angle.*;

/**
 * TeleOp to be used for the robot
 * @author Jeffrey Tvedt
 */
@TeleOp(name="Jeffrey TeleOp")
public class JeffreyTeleOp extends LinearOpMode {
    Sybot robot;

    // boolean values indicating button state
    ControlMode control = ControlMode.MULTIPLAYER;

    boolean a, b, x;
    boolean a2, b2, x2, y2;
    boolean uPad2, dPad2, lPad2, rPad2;

    @Override
    public void runOpMode() {
        robot = new Sybot(this, Sybot.OpModeType.TELEOP);
        robot.setClaw(false);

        while (opModeIsActive()) {
            runLoop();
        }

        robot.stop();
        telemetry.update();
    }

    public void runLoop() {
        // Determine which control mode to run
        if (lTrigger(1) && rTrigger(1) && gamepad1.right_bumper) {
            if (gamepad1.y) control = ControlMode.MULTIPLAYER;
            if (gamepad1.x) control = ControlMode.CONE_CYCLE;
        }

        switch (control) {
            case CONE_CYCLE:
                coneCycleLoop();
                break;
            case MULTIPLAYER:
                standardLoop();
                break;
        }
    }

    // Standard loop that involves two players
    public void standardLoop() {
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

        // Move only in cardinal directions if pressing Y
        if (gamepad1.y) stickAngle = Math.round(stickAngle * 2/Math.PI) * Math.PI/2;

        // Turn control
        if (gamepad1.left_bumper) robot.resetAngle();
        if (lTrigger(1)) {
            double roundedAngle = Math.round(robot.getAngle() * 2/Math.PI) * Math.PI/2;
            if (robot.getAngle() != roundedAngle) turn = 0.3 * (robot.getAngle() < roundedAngle ? -1 : 1);
        }

        // Plug in numbers
        robot.teleDrive(stickAngle, magnitude, turn, multiplier);

        // P2 Crane subsystem
        if (gamepad2.dpad_up && !uPad2) robot.setSlides(-Sybot.SLIDE_HIGH);
        if (gamepad2.dpad_right && !rPad2) robot.setSlides(-2000);
        if (gamepad2.dpad_down && !dPad2) robot.dropSlides();
        if (gamepad2.dpad_left && !lPad2) robot.lockSlides();

        // Manual slide fine tuning
        if (gamepad2.right_stick_y != 0) robot.manualSlides = true;
        if (robot.manualSlides) robot.moveSlides(gamepad2.right_stick_y * 0.5);
        if (gamepad2.left_bumper) robot.resetSlides();
        if (lTrigger(2)) robot.moveSlides(1.0);

        // Claw subsystem
        if (gamepad2.a && !a2) robot.toggleClaw(); // Regular cones
        if (gamepad2.b && !b2) robot.setClaw(0.25); // Capstone
        if (gamepad2.x && !x2) robot.slideRelease = !robot.slideRelease;

        resetButtons();

        // telemetry.addData("Robot angle", robot.getAngle());

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

        // telemetry.addData("Debug int", robot.debugInt);
        // telemetry.addData("Debug double", robot.debugDouble);
        telemetry.addData("Counter", robot.counter);

        // telemetry.addData("Claw state", robot.pinch ? "closed" : "open");

        telemetry.update();
    }

    // Loop for cycling cones on junction
    public void coneCycleLoop() {
        // Stick Input
        double stickAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x);
        double magnitude = (rTrigger(1) ? 0.35 : 1);
        stickAngle = Math.round(stickAngle * 2/Math.PI) * Math.PI/2;

        double turn = 0;
        double roundedAngle = Math.round(robot.getAngle() * 2/Math.PI) * Math.PI/2;
        if (robot.getAngle() != roundedAngle) turn = 0.3 * (robot.getAngle() < roundedAngle ? -1 : 1);

        robot.teleDrive(stickAngle, magnitude, turn);

        if (gamepad1.x && !x) robot.toggleClaw();
        if (gamepad1.a && !a) {
            robot.setClaw(false);
            robot.dropSlides();
        }
        if (gamepad1.b && !b) {
            robot.setClaw(true);
            robot.setSlides(-4150);
        }

        if (gamepad1.right_stick_y != 0) robot.manualSlides = true;
        if (robot.manualSlides) robot.moveSlides(gamepad1.right_stick_y * 0.7);
    }

    public enum ControlMode {
        CONE_CYCLE,
        MULTIPLAYER,
        AUTOMATIC
    }

    public void resetButtons() {
        // Reset P1 buttons
        a = gamepad1.a;
        b = gamepad1.b;
        x = gamepad1.x;

        // Reset P2 buttons
        a2 = gamepad2.a;
        b2 = gamepad2.b;
        y2 = gamepad2.y;
        x2 = gamepad2.x;
        uPad2 = gamepad2.dpad_up;
        dPad2 = gamepad2.dpad_down;
        lPad2 = gamepad2.dpad_left;
        rPad2 = gamepad2.dpad_right;
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