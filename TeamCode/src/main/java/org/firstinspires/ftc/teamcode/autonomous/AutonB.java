package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotMethods;

@Disabled
@Autonomous(name="Test Autonomous B", group="test")
public class AutonB extends LinearOpMode {
    public RobotMethods robot;

    @Override
    public void runOpMode() {
        robot = new RobotMethods(this);

        // Autonomous processes go here
        // TODO test the CvPipeline
    }
}