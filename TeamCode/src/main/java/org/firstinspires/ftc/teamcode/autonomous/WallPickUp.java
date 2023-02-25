package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.Sybot;

/**
 * In case slides aren't working this is a parking only auton using april tags
 * @author Tyler Philip & Jeffrey Tvedt
 */
@Autonomous(name="Wall Pickup")
public class WallPickUp extends LinearOpMode{
    public Sybot robot;
    @Override
    public void runOpMode() {
        Sybot.setImplementation(Sybot.CvImplementation.APRIL_TAGS);
        robot = new Sybot(this, Sybot.OpModeType.AUTONOMOUS);
        robot.setClaw(false);
        robot.setSlides(-1500);
        robot.setClaw(true);
        robot.waitForSlides();
        robot.setSlides(-4310);
        robot.drive(-50);
        robot.setClaw(false);
    }


}
