package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.Angle;
import org.firstinspires.ftc.teamcode.util.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.Sybot;

@Autonomous(name="Test Autonomous A", group="test")
public class AutonA extends LinearOpMode {
    public Sybot robot;

    @Override
    public void runOpMode() {
        robot = new Sybot(this);
        robot.setDriveUnit(DistanceUnit.TILES);

        robot.drive(3.5);
        robot.drive(-3.5);
    }
}