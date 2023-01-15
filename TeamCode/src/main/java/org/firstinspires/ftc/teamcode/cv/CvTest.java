package org.firstinspires.ftc.teamcode.cv;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.SyBot;

@Autonomous(name="CV Test", group="test")
public class CvTest extends LinearOpMode {
    public SyBot robot;

    @Override
    public void runOpMode() {
        robot = new SyBot(this, SyBot.OpModeType.AUTONOMOUS);
        telemetry.addData("Zone", robot.pipeline.getZone());
        telemetry.update();
    }
}