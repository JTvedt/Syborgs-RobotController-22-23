package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.Angle;
import org.firstinspires.ftc.teamcode.util.Sybot;

@Autonomous(name="Test Autonomous A", group="test")
public class AutonA extends LinearOpMode {
    public Sybot robot;

    @Override
    public void runOpMode() {
        robot = new Sybot(this);

        robot.spinTo(-45);
        robot.spinTo(90);
        telemetry.addData("Angle", robot.getAngle());
        sleep(400);
        robot.spinTo(-130);
        robot.spinTo(90);
        telemetry.addData("Angle", robot.getAngle());
        sleep(400);
        robot.spinTo(-90);
        robot.spinTo(90);
        telemetry.addData("Angle", robot.getAngle());
        telemetry.update();
        sleep(400);
        sleep(200000);
    }
}