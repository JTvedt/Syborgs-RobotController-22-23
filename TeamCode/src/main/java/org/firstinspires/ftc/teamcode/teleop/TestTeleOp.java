package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.Sybot;

@TeleOp(name="Test TeleOp")
public class TestTeleOp extends LinearOpMode {
    public Sybot robot;

    @Override
    public void runOpMode() {
        robot = new Sybot(this, Sybot.OpModeType.TELEOP);

        while (opModeIsActive()) {

        }

        telemetry.addData("End", "End");
        telemetry.update();
        sleep(1000);
    }
}
