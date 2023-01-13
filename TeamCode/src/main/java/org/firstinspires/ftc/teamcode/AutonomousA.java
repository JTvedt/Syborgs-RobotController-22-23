package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Left Hand Autonomous")
public class AutonomousA extends LinearOpMode {
    public RobotMethods robot;

    @Override
    public void runOpMode() {
        robot = new RobotMethods(this);

        // Autonomous processes go here
        robot.drive(60);
        robot.strafe(60);
        robot.drive(-60);
        robot.strafe(-60);
    }

    // 0 for red, 1 for green, 2 for blue
    public void park(int parkingSpot) {

    }
}