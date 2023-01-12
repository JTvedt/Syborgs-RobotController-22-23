package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Right Hand Autonomous")
public class AutonomousB extends LinearOpMode {
    public Robot robot;

    @Override
    public void runOpMode() {
        robot = new Robot(this);

        // Autonomous processes go here
        robot.setSlides(-2840);
        sleep(8000);
        robot.setSlides(0);
        sleep(8000);
    }
}
