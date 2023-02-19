package org.firstinspires.ftc.teamcode.deprecated;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.checkerframework.framework.qual.DefaultInUncheckedCodeFor;
import org.firstinspires.ftc.teamcode.util.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.Sybot;
/**
 * In case slides aren't working this is a parking only auton using april tags
 * @author Tyler Philip & Alex Malladi & Jeffrey Tvedt
 */
@Deprecated
@Disabled
@Autonomous(name="2C Left Autonomous AT")
public class DoubleLeftAT extends LinearOpMode {
    public Sybot robot;

    @Override
    public void runOpMode() {
        Sybot.setImplementation(Sybot.CvImplementation.APRIL_TAGS);
        robot = new Sybot(this, Sybot.OpModeType.AUTONOMOUS, Sybot.StartSide.LEFT);
        robot.mirrorStrafe = true;

        robot.setClaw(true);
        int parkZone = robot.retrieveZone();

        // Autonomous processes go here
        // Everything in this section is mirrored
        robot.setSlides(-4310); //drop slides
        robot.drive(40); //forward to drop off cone
        robot.drive(-9);//back to tile
        robot.strafe(-18);//strafe right
        robot.drive(2);
        robot.strafe(-20);
        robot.drive(3);//forward slightly
        robot.setSlides(0);
        robot.setClaw(false);
        robot.waitForSlides();
        robot.drive(-3);
        robot.strafe(15);
        robot.drive(22);//With spin drive drive replaces strafe due to IMU influence
        robot.spin(90);
        robot.strafe(-56);
//        robot.drive(-34);
//        robot.strafe(15);
//        robot.setSlides(-4500);
//        robot.setClaw(false);




//        park(parkZone);
//        robot.waitForSlides();
//        sleep(800);
    }

    // 0 for red, 1 for green, 2 for blue
    public void park(int parkingSpot) {
        robot.mirrorStrafe = false;
        if (parkingSpot == 9) robot.strafe(-26);
        else if (parkingSpot == 11) robot.strafe(26);
    }
}