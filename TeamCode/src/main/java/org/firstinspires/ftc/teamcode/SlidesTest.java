package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Configure Slides")
public class SlidesTest extends LinearOpMode {
    Robot robot;
    int slideHeight;
    boolean a, b;

    @Override
    public void runOpMode() {
        robot = new Robot(this);
        while (opModeIsActive()) runLoop();
    }

    public void runLoop() {
        double multiplier = (gamepad1.left_trigger > 0.5 ? 10d : 1d) / (gamepad1.right_trigger > 0.5 ? 10d : 1d);

        if (gamepad1.a && !a) slideHeight += 100 * multiplier;
        if (gamepad1.b && !b) slideHeight -= 100 * multiplier;

        robot.setSlides(slideHeight);

        a = gamepad1.a;
        b = gamepad1.b;

        telemetry.addData("Slide Height:", slideHeight);
        telemetry.addData("Left Position:", robot.leftSlide.getCurrentPosition());
        telemetry.addData("Right Position:", robot.rightSlide.getCurrentPosition());
        telemetry.update();
    }
}
