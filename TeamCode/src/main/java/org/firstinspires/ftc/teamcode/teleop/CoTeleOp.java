package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.Angle;
import org.firstinspires.ftc.teamcode.util.Sybot;
import org.firstinspires.ftc.teamcode.util.Structured;

@TeleOp(name="Two Player TeleOp")
public class CoTeleOp extends LinearOpMode implements Structured {
    Sybot robot;

    boolean a2, b2, x2, y2;

    boolean uPad, dPad, lPad, rPad;
    boolean uPad2, dPad2, lPad2, rPad2;

    boolean rigidMove = false, smoothAngle =false;

    @Override
    public void runOpMode() {
        robot = new Sybot(this, Sybot.OpModeType.TELEOP);

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
        // Take input
        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        double stickAngle = Math.atan2(drive, strafe);
        double magnitude = Math.hypot(drive, strafe);
        double multiplier = 0.6 * (gamepad1.a ? 1.4 : 1) * (gamepad1.right_trigger > .5 ? 0.5 : 1);

        // Toggles settings
        if (gamepad1.dpad_left && !lPad) rigidMove = !rigidMove;
        if (gamepad1.dpad_up && !uPad) smoothAngle = !smoothAngle;

        if (gamepad1.dpad_right && !rPad) robot.resetAngle();
        if (gamepad1.dpad_down && !dPad) robot.resetSlides();

        if (rigidMove) stickAngle = Angle.round(stickAngle);
        if (smoothAngle && turn == 0) turn = robot.smoothAngle();

        // Plug in numbers
        robot.teleDrive(stickAngle, magnitude, turn, multiplier);
    }

    @Override
    public void slideSubsystem() {
        if (gamepad2.y && !y2) robot.setSlides(Sybot.SLIDE_HIGH_TICKS);
        if (gamepad2.x && !x2) robot.dropSlides();

        if (gamepad2.left_trigger > .5 || gamepad2.right_trigger > .5)
            robot.moveSlides((gamepad2.left_trigger + gamepad2.right_trigger)/2);
        if (gamepad2.left_bumper || gamepad2.right_bumper)
            robot.moveSlides((gamepad2.left_bumper ? .5 : 0) + (gamepad2.right_bumper ? .5 : 0));
    }

    @Override
    public void clawSubsystem() {
        if (gamepad2.a && !a2) robot.toggleClaw();
    }

    @Override
    public void resetButtons() {
        uPad = gamepad1.dpad_up;
        dPad = gamepad1.dpad_down;
        lPad = gamepad1.dpad_left;
        rPad = gamepad1.dpad_right;

        a2 = gamepad2.a;
        b2 = gamepad2.b;
        x2 = gamepad2.x;
        y2 = gamepad2.y;

        uPad2 = gamepad2.dpad_up;
        dPad2 = gamepad2.dpad_down;
        lPad2 = gamepad2.dpad_left;
        rPad2 = gamepad2.dpad_right;
    }

    @Override
    public void telemetryConsole() {

    }
}
