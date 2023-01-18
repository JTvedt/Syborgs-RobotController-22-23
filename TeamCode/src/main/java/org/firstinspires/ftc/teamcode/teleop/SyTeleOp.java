package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.Sybot;

// Thanks Arjun. Go do your homework

/**
 * TeleOp to be used for the robot
 * @author Jeffrey Tvedt
 */
@TeleOp(name="Jeffrey TeleOp")
public class SyTeleOp extends LinearOpMode {
    public Sybot robot;

    // boolean values indicating button state
    public ControlMode control = ControlMode.MULTIPLAYER;

    public boolean a, b;
    public boolean a2, b2, x2, y2;
    public boolean uPad2, dPad2, lPad2, rPad2;

    @Override
    public void runOpMode() {
        robot = new Sybot(this, Sybot.OpModeType.TELEOP);
        robot.setClaw(false);

        while (opModeIsActive()) {
            runLoop();
        }
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

        // TODO find a way to automate this process
        // Reset P1 buttons
        a = gamepad1.a;
        b = gamepad1.b;

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

    // Loop for cycling cones on junction
    public void coneCycleLoop() {

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

        double roundedAngle = Math.round(robot.getAngle() * 2/Math.PI) * Math.PI/2;

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
        if (gamepad1.y) stickAngle = roundedAngle;

        // Turn control
        if (gamepad1.left_bumper) robot.resetAngle();
        if (lTrigger(1) && robot.getAngle() != roundedAngle) {
            turn = 0.3 * (robot.getAngle() < roundedAngle ? -1 : 1);
        }

        // Plug in numbers
        robot.teleDrive(stickAngle, magnitude, turn, multiplier);

        // P2 Crane subsystem
        if (gamepad2.dpad_up && !uPad2) robot.setSlides(-4300);
        if (gamepad2.dpad_right && !rPad2) robot.setSlides(-2000);
        if (gamepad2.dpad_down && !dPad2) robot.setSlides(-100);
        if (gamepad2.dpad_left && !lPad2) robot.setSlides(robot.slidePosition()); // Locks slides position

        // Manual slide fine tuning
        if (gamepad2.x && !x2) robot.setSlides(robot.slideTarget() + (rTrigger(2) ? 35 : 100));
        if (gamepad2.y && !y2) robot.setSlides(robot.slideTarget() - (rTrigger(2) ? 35 : 100));
        if (gamepad2.right_stick_y != 0) robot.manualSlides = true;
        if (robot.manualSlides) robot.moveSlides(gamepad2.right_stick_y * 0.5);
        if (gamepad2.left_bumper) robot.resetSlides();
        if (lTrigger(2)) robot.moveSlides(1.0);

        // Claw subsystem
        if (gamepad2.a && !a2) robot.toggleClaw(); // Regulr cones
        if (gamepad2.b && !b2) robot.setClaw(0.25); // Capstone

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

    public enum ControlMode {
        CONE_CYCLE,
        MULTIPLAYER,
        AUTOMATIC
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