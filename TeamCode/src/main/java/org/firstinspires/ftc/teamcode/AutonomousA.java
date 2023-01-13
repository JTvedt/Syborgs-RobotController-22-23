package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.cv.CvPipeline;

@Autonomous(name="Left Hand Autonomous")
public class AutonomousA extends LinearOpMode {
    public RobotMethods robot;
    public CvPipeline cv;

    @Override
    public void runOpMode() {
        robot = new RobotMethods(this);

        // Autonomous processes go here
        robot.drive(36);
        robot.drive(-36);
    }

    // 0 for red, 1 for green, 2 for blue
    public void park(int parkingSpot) {

    }
}