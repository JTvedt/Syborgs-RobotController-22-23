package org.firstinspires.ftc.teamcode.cv;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.Sybot;

@Autonomous(name="CV Test", group="test")
public class CvTest extends LinearOpMode {
    public Sybot robot;

    @Override
    public void runOpMode() {
        robot = new Sybot(this, Sybot.OpModeType.AUTONOMOUS);
        telemetry.addData("Zone", robot.getZone());
        telemetry.update();
    }
}