package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.Sybot;
import org.firstinspires.ftc.teamcode.util.TeleOpStructured;

@TeleOp(name="2P TeleOp")
public class CoTeleOp extends LinearOpMode implements TeleOpStructured {
    private Sybot robot;

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

    }

    @Override
    public void slideSubsystem() {

    }

    @Override
    public void clawSubsystem() {

    }

    @Override
    public void resetButtons() {

    }

    @Override
    public void telemetryConsole() {

    }
}
