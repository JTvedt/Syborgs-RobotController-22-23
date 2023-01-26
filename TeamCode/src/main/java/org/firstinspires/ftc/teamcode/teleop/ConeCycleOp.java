package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.Sybot;
import org.firstinspires.ftc.teamcode.util.Structured;

@TeleOp(name="Singleplayer TeleOp")
public class ConeCycleOp extends LinearOpMode implements Structured {
    public Sybot robot;
    boolean a, b, x, y;
    boolean uPad, dPad, lPad, rPad;
    boolean lb, rb;
    boolean rigidMove = false;
    boolean correctSpin = false;

    @Override
    public void runOpMode() {
        robot = new Sybot(this, Sybot.OpModeType.TELEOP);
        robot.setClaw(false);

        while (opModeIsActive()) {
            driveTrain();
            slideSubsystem();
            clawSubsystem();
            resetButtons();
            telemetryConsole();
        }

        robot.stop();
    }

    @Override
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
        if (correctSpin && turn == 0) turn = robot.smoothAngle();

        if (gamepad1.right_stick_x == 0) robot.teleDrive(stickAngle, magnitude * multiplier, turn);
        else robot.teleDrive(stickAngle, magnitude, turn, multiplier);
    }

    @Override
    public void slideSubsystem() {
        if (gamepad1.dpad_up && !uPad) robot.setSlides(Sybot.SLIDE_HIGH_TICKS);
        if (gamepad1.dpad_right && !rPad) robot.setSlides(-2000);
        if (gamepad1.dpad_down && !dPad) robot.dropSlides();

        if (gamepad1.x) robot.moveSlides(gamepad1.left_trigger/2 + .5);
        if (gamepad1.y) robot.moveSlides(-gamepad1.left_trigger/2 + .5);
    }

    @Override
    public void clawSubsystem() {
        if (gamepad1.a && !a) robot.toggleClaw();
        if (gamepad1.b && !b) robot.pinchSlide();
    }

    @Override
    public void resetButtons() {
        a = gamepad1.a;
        b = gamepad1.b;
        x = gamepad1.x;
        y = gamepad1.y;

        uPad = gamepad1.dpad_up;
        dPad = gamepad1.dpad_down;
        lPad = gamepad1.dpad_left;
        rPad = gamepad1.dpad_right;

        lb = gamepad1.left_bumper;
        rb = gamepad1.right_bumper;
    }

    @Override
    public void telemetryConsole() {}
}