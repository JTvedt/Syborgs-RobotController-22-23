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
        robot.toggleClaw(true);
        sleep(670);

        // Autonomous processes go here
        robot.setSlides(-4130);
        robot.drive(56);
        robot.strafe(-12);
        robot.waitForSlides();
        robot.drive(3);
        robot.setSlides(730);
        sleep(300);
        robot.toggleClaw();
        robot.drive(-3);
        robot.strafe(12);
        robot.spin(-90);
        robot.strafe(24);
        robot.toggleClaw();
//        robot.setSlides(-4130);
//        sleep(600);
//        robot.strafe(-24);
//        robot.spin(90);
//        robot.strafe(-12);
//        robot.drive(3);
//        robot.setSlides(0);
//        sleep(300);
//        robot.toggleClaw();
//        robot.drive(-3);
//        robot.strafe(12);
//        robot.drive(-26);
//
//        park(2);
//        robot.waitForSlides();
//        sleep(800);
    }

    // 0 for red, 1 for green, 2 for blue
    public void park(int parkingSpot) {
        if (parkingSpot == 0) robot.strafe(-24);
        else if (parkingSpot == 1) robot.strafe(24);
    }
}