package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotMethods;
import org.firstinspires.ftc.teamcode.cv.CvPipeline;

@Disabled
@Autonomous(name="2C Right Autonomous")
public class LeftDouble extends LinearOpMode {
    public RobotMethods robot;
    public CvPipeline cv;

    @Override
    public void runOpMode() {
        robot = new RobotMethods(this);
        robot.toggleClaw(true);
        sleep(670);

        // Autonomous processes go here

    }

    // 0 for red, 1 for green, 2 for blue
    public void park(int parkingSpot) {
        if (parkingSpot == 0) robot.strafe(-24);
        else if (parkingSpot == 1) robot.strafe(24);
    }
}