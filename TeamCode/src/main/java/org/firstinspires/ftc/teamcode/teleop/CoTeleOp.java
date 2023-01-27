package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.Angle;
import org.firstinspires.ftc.teamcode.util.Controller;
import org.firstinspires.ftc.teamcode.util.Sybot;
import org.firstinspires.ftc.teamcode.util.Structured;

@TeleOp(name="Two Player TeleOp")
public class CoTeleOp extends LinearOpMode implements Structured {
    Controller controller;
    Sybot robot;

    boolean rigidMove = false, smoothAngle =false;

    @Override
    public void runOpMode() {
        controller = new Controller(gamepad1, gamepad2);
        robot = new Sybot(this, Sybot.OpModeType.TELEOP);

        while (opModeIsActive()) {
            driveTrain();
            slideSubsystem();
            clawSubsystem();
            controller.update();
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
        if (controller.press(1, "DL")) rigidMove = !rigidMove;
        if (controller.press(1, "DU")) smoothAngle = !smoothAngle;

        if (controller.press(1, "DR")) robot.resetAngle();
        if (controller.press(1, "DL")) robot.resetSlides();

        if (rigidMove) stickAngle = Angle.round(stickAngle);
        if (smoothAngle && turn == 0) turn = robot.smoothAngle();

        // Plug in numbers
        robot.teleDrive(stickAngle, magnitude, turn, multiplier);
    }

    @Override
    public void slideSubsystem() {
        if (controller.press(2, "Y")) robot.setSlides(Sybot.SLIDE_HIGH_TICKS);
        if (controller.press(2, "X")) robot.dropSlides();

        if (robot.manualSlides) robot.moveSlides(0);
        if (gamepad1.left_trigger > .5) robot.moveSlides(1);
        if (gamepad1.right_trigger > .5) robot.moveSlides(.5);
        if (gamepad1.left_bumper) robot.moveSlides(-1);
        if (gamepad1.right_bumper) robot.moveSlides(-.5);
    }

    @Override
    public void clawSubsystem() {
        if (controller.press(2, "A")) robot.toggleClaw();
        if (controller.press(2, "LB")) Sybot.CLOSE_CLAW -= 0.02;
        if (controller.press(2, "RB")) Sybot.CLOSE_CLAW += 0.02;
    }

    @Override
    public void telemetryConsole() {

    }
}
