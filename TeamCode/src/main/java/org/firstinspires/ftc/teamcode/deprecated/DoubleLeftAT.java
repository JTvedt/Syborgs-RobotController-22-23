package org.firstinspires.ftc.teamcode.deprecated;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.Sybot;
/**
 * In case slides aren't working this is a parking only auton using april tags
 * @author Tyler Philip & Alex Malladi & Jeffrey Tvedt
 */

@Disabled
@Deprecated //Jeffrey changed radians :(
@Autonomous(name="2C Left Autonomous AT ")
public class DoubleLeftAT extends LinearOpMode {
    public Sybot robot;



    @Override
    public void runOpMode() {
        Sybot.setImplementation(Sybot.CvImplementation.APRIL_TAGS);
        robot = new Sybot(this, Sybot.OpModeType.AUTONOMOUS, Sybot.StartSide.LEFT);
        new Thread(new OutputAngle(robot)).start();
        robot.setDriveType(Sybot.DriveType.POV);

        robot.setClaw(true);
        int parkZone = robot.retrieveZone();

        // Autonomous processes go here
        // Everything in this section is mirrored
        robot.setSlides(-4310); //Bring up slides
        robot.drive(37); //forward to drop off cone
        robot.drive(-7);//back to tile
        robot.strafe(18);//strafe right
        robot.drive(2); //Adjust forward
        robot.strafe(20); //Strafe towards junction
        robot.drive(3);//forward slightly to junction
        robot.setSlides(0); //Bring slides down
        robot.setClaw(false); // Open claw
        robot.waitForSlides(); // Waits for slides to finish

        //Start of second cone pickup
        robot.drive(-3); //backup slightly
        robot.strafe(-15); //Strafe left
        robot.drive(24);//With spin drive drive replaces strafe due to IMU influence
        robot.spin(90); //Spin towards cones
        robot.setDriveType(Sybot.DriveType.DIRECTIONAL); //Inverts values and resets IMU
        robot.strafe(1.8); //Strafe left
        robot.drive(52); // Drive up to cones
        sleep(300);//will be removed at some point for efficiency
        robot.setSlides(-1000); //Pickup Top Cone On Stack
        robot.waitForSlides();
        robot.strafe(2);
//        robot.drive(3);
        robot.setClaw(true);
        robot.setDriveType(Sybot.DriveType.POV);
        robot.setSlides(-1500);
        robot.waitForSlides();
        robot.setSlides(-4310);
        robot.strafe(50);
//        robot.strafe(-20);




//        park(parkZone);
//        robot.waitForSlides();
//        sleep(800);
    }

    // 0 for red, 1 for green, 2 for blue
    public void park(int parkingSpot) {
        robot.mirrorDirection = false;
        if (parkingSpot == 9) robot.strafe(-26);
        else if (parkingSpot == 11) robot.strafe(26);
    }


    public class OutputAngle implements Runnable {
        private Sybot robot;
        public OutputAngle(Sybot robot) {
            this.robot = robot;
        }

        @Override
        public void run() {
            while (robot.enableThreads) {
                telemetry.addLine("Angle: " + robot.getAngle());
                robot.rest(200);
            }
        }
    }
}