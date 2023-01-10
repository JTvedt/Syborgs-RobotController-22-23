package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Autonomous(name="Left Hand Autonomous")
public class AutonomousA extends AutonomousBase {
    @Override
    public void runOpMode() {
        waitForStart();
        telemetry.addData("Number", foo());
        telemetry.update();
        sleep(1000);
    }
}
