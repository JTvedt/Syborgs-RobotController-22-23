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
        robot.drive(100);
        robot.spin(90);
        robot.setSlides(3800);
        robot.drive(8);
        robot.waitForSlides();
        robot.toggleClaw();
        robot.setSlides(0);
        robot.drive(-8);
        robot.spin(-100);
        robot.drive(-60);
        robot.strafe(60);
        robot.waitForSlides();
    }
}
