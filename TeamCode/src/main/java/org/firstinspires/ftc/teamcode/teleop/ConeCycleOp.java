package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.Angle;
import org.firstinspires.ftc.teamcode.util.Sybot;

@TeleOp(name="Cone Cycle TeleOp")
public class ConeCycleOp extends LinearOpMode {
    public Sybot robot;
    boolean a, b, x;
    boolean lb, rb;
    boolean rigidMove = false;
    boolean correctSpin = false;

    @Override
    public void runOpMode() {
        robot = new Sybot(this, Sybot.OpModeType.TELEOP);
        robot.setClaw(false);

        while (opModeIsActive()) {
            runLoop();
        }

        robot.stop();
    }

    public void runLoop() {
        driveTrain();
        slideSubsystem();
        clawSubsystem();
        resetButtons();
        telemetryConsole();
    }

    public void driveTrain() {
        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x * 0.75;

        double stickAngle = Math.atan2(drive, strafe);
        double magnitude = Math.hypot(drive, strafe);
        double multiplier = 0.6 * (gamepad1.b ? 1.4 : 1) * (gamepad1.right_trigger > 0.5 ? 0.35 : 1);

        if (gamepad1.left_bumper && !lb) rigidMove = !rigidMove;
        if (gamepad1.right_bumper && !rb) correctSpin = !correctSpin;

        if (rigidMove) stickAngle = Math.round(stickAngle * 2/Math.PI) * Math.PI/2;
        if (correctSpin && turn == 0) {
            double angleDiff = robot.getAngleDifference(Angle.roundAngle(robot.getAngle()));
            if (Math.abs(angleDiff) < Math.PI/60) turn = 0;
            else if (Math.abs(angleDiff) < Math.PI/36) turn = 0.1 * Math.signum(angleDiff);
            else turn = 0.3 * Math.signum(angleDiff);
        }

        if (gamepad1.right_stick_x == 0) robot.teleDrive(stickAngle, magnitude * multiplier, turn);
        else robot.teleDrive(stickAngle, magnitude, turn, multiplier);
    }

    public void slideSubsystem() {
        if (gamepad1.right_stick_y != 0) robot.manualSlides = true;
        if (robot.manualSlides) robot.moveSlides(gamepad1.right_stick_y * (gamepad1.left_trigger/2 + .5));
    }

    public void clawSubsystem() {
        if (gamepad1.x && !x) robot.toggleClaw();
        if (gamepad1.a && !a) robot.pinchSlide();
    }

    public void resetButtons() {
        a = gamepad1.a;
        b = gamepad1.b;
        x = gamepad1.x;
        lb = gamepad1.left_bumper;
        rb = gamepad1.right_bumper;
    }

    public void telemetryConsole() {}
}