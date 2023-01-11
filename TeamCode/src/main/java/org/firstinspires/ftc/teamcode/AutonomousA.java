package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Disabled
@Autonomous(name="Left Hand Autonomous")
public class AutonomousA extends LinearOpMode {
    public RobotMethods robot;

    @Override
    public void runOpMode() {
        robot = new RobotMethods(this);
        waitForStart();

        telemetry.addData("Number", robot.foo());
        telemetry.update();
        sleep(1000);
    }
}
