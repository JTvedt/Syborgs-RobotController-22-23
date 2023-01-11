package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Disabled
@Autonomous(name="Right Hand Autonomous")
public class AutonomousB extends LinearOpMode {
    public RobotMethods robot;

    @Override
    public void runOpMode() {
        robot = new RobotMethods(this);
        waitForStart();

        // Autonomous processes go here
        telemetry.addData("Angle: ", robot.getAngle());
        telemetry.update();
        sleep(1000);
        telemetry.addData("Angle: ", robot.getAngle());
        telemetry.update();
        sleep(1000);
    }
}
