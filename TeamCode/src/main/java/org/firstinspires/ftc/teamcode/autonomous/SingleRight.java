package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.SyBot;

@Autonomous(name="1C Right Autonomous")
public class SingleRight extends LinearOpMode {
    public SyBot robot;

    @Override
    public void runOpMode() {
        robot = new SyBot(this, SyBot.OpModeType.AUTONOMOUS);

        robot.toggleClaw(true);
        sleep(400);
        int parkZone = robot.pipeline.getZone();
        telemetry.addData("Parking in", parkZone);
        telemetry.update();
        sleep(600);

        // Autonomous processes go here
        robot.setSlides(-4130);
        robot.drive(56);
        robot.strafe(-12);
        robot.waitForSlides();
        robot.drive(3);
        robot.setSlides(0);
        robot.rest();
        robot.toggleClaw();
        robot.rest();
        robot.drive(-3);
        robot.strafe(12);
        robot.drive(-26);

        park(parkZone);
        robot.waitForSlides();
        sleep(800);
    }

    // 0 for red, 1 for green, 2 for blue
    public void park(int parkingSpot) {
        if (parkingSpot == 0) robot.strafe(-26);
        else if (parkingSpot == 2) robot.strafe(26);
    }
}