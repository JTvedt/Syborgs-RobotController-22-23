package org.firstinspires.ftc.teamcode.cv;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotMethods;

@Autonomous(name="CV Test", group="test")
public class CvTest extends LinearOpMode {
    public RobotMethods robot;

    @Override
    public void runOpMode() {
        robot = new RobotMethods(this, RobotMethods.OpModeType.AUTONOMOUS);
        telemetry.addData("Zone", robot.pipeline.getZone());
        telemetry.update();
    }
}