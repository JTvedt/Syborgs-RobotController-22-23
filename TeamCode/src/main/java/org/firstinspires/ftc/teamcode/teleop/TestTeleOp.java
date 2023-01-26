package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.Controller;
import org.firstinspires.ftc.teamcode.util.Sybot;

@TeleOp(name="Test TeleOp")
public class TestTeleOp extends LinearOpMode {
    public Controller controller;
    public Sybot robot;

    @Override
    public void runOpMode() {
        controller = new Controller(gamepad1);
        robot = new Sybot(this, Sybot.OpModeType.TELEOP);

        while (opModeIsActive()) {
            if (controller.press("A")) {
                robot.toggleClaw();
            }

            if (controller.hold("X")) {
                robot.moveSlides(0.7);
            }

            if (controller.hold("Y")) {
                robot.moveSlides(-0.7);
            }

            controller.update();

            telemetry.addData("toString", gamepad1.toString());
            telemetry.addData("contains b", gamepad1.toString().contains(" b "));
            telemetry.update();
        }

        telemetry.addData("End", "End");
        telemetry.update();
        sleep(1000);
    }
}
