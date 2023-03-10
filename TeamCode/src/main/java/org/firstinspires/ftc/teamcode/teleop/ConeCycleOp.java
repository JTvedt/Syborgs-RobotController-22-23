package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.Angle;
import org.firstinspires.ftc.teamcode.util.Controller;
import org.firstinspires.ftc.teamcode.util.Sybot;
import org.firstinspires.ftc.teamcode.util.Structured;

@TeleOp(name="Singleplayer TeleOp")
public class ConeCycleOp extends LinearOpMode implements Structured {
    public Controller controller;
    public Sybot robot;

    boolean rigidMove, smoothAngle = false;

    @Override
    public void runOpMode() {
        controller = new Controller(gamepad1);
        robot = new Sybot(this, Sybot.OpModeType.TELEOP);
        robot.setClaw(false);

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
        double drive = -gamepad1.left_stick_y * 0.8;
        double strafe = gamepad1.left_stick_x * 0.8;
        double turn = gamepad1.right_stick_x * 0.75;

        double stickAngle = Math.atan2(drive, strafe);
        double magnitude = Math.hypot(drive, strafe);
        double multiplier = 0.8 * (gamepad1.x ? 1.25 : 1) * (gamepad1.right_trigger > 0.5 ? 0.35 : 1);

        if (controller.press("LB"))
            rigidMove = !rigidMove;
        if (controller.press("RB"))
            smoothAngle = !smoothAngle;
        if (controller.press("LS"))
            robot.resetAngle();

        if (rigidMove)
            stickAngle = Math.round(stickAngle * 2/Math.PI) * Math.PI/2;
        if (smoothAngle && turn == 0)
            turn = -robot.smoothAngle();

        if (gamepad1.right_stick_x == 0)
            robot.teleDrive(stickAngle, magnitude * multiplier, turn);
        else
            robot.teleDrive(stickAngle, magnitude, turn, multiplier);
    }

    @Override
    public void slideSubsystem() {
        if (controller.press("DU"))
            robot.setSlides(Sybot.SLIDE_HIGH_TICKS);
        if (controller.press("DR"))
            robot.setSlides(-500);
        if (controller.press("DD"))
            robot.setSlides(0);

        if (controller.press("RS"))
            robot.resetSlides();

        if (robot.manualSlides)
            robot.moveSlides(0);
        if (gamepad1.x)
            robot.setSlides(robot.slideTarget() + 20);
        if (gamepad1.y)
            robot.setSlides(robot.slideTarget() - 20);
    }

    @Override
    public void clawSubsystem() {
        if (controller.press("A"))
            robot.toggleClaw();
        if (controller.press("B"))
            robot.pinchSlide();
    }

    @Override
    public void telemetryConsole() {
        String settings = "";
        if (smoothAngle)        settings += "smooth_angle ";
        if (rigidMove)          settings += "rigid_move ";
        if (robot.manualSlides) settings += "manual_slides ";
        telemetry.addData("Settings", settings);

        telemetry.addData("Robot Angle", robot.getAngle());
        telemetry.addData("Left Slide", robot.leftSlide.getCurrentPosition());
        telemetry.addData("Right Slide", robot.rightSlide.getCurrentPosition());
        telemetry.addData("Slide Target", robot.slideTarget());
        telemetry.addData("Slide Delta", robot.slideDelta);
        telemetry.addData("Claw State", robot.pinch ? "CLOSE" : "OPEN");

        telemetry.update();
    }
}