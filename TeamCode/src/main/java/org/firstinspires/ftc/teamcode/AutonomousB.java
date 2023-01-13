package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Right Hand Autonomous")
public class AutonomousB extends LinearOpMode {
    public RobotMethods robot;

    @Override
    public void runOpMode() {
        robot = new RobotMethods(this);

        // Autonomous processes go here

    }
}
