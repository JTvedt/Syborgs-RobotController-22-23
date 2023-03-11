package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.Sybot;

/**
 * In case slides aren't working this is a parking only auton using april tags
 * @author Tyler Philip & Jeffrey Tvedt
 */
@Autonomous(name="Park Auton")
public class ParkOnlyAuton extends LinearOpMode{
    public Sybot robot;
    @Override
    public void runOpMode() {
        Sybot.setImplementation(Sybot.CvImplementation.APRIL_TAGS);
        robot = new Sybot(this, Sybot.OpModeType.AUTONOMOUS);
        robot.setDriveUnit(DistanceUnit.TILES);
        robot.retrieveZone(); //gets AprilTags value

        robot.drive(1.25);
        sleep(400);
        if(robot.parkZone == 9){
            robot.strafe(-1);
            telemetry.addData("PARKING IN","LEFT");
            telemetry.update();
        }else if(robot.parkZone == 11){
            robot.strafe(1);
            telemetry.addData("PARKING IN","RIGHT");
            telemetry.update();
        }
    }


}
