package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Left Hand Autonomous")
public class AutonomousA extends LinearOpMode {
    public Robot robot;

    @Override
    public void runOpMode() {
        robot = new Robot(this);

        // Autonomous Processes go here
        for (int i = 0; i < 10; i++) {
            sleep(500);
            robot.toggleClaw();
        }

        sleep(1000);
    }

    // 0 for red, 1 for green, 2 for blue
    public void park(int parkingSpot) {

    }
}