package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Autonomous(name="Right Hand Autonomous")
public class AutonomousB extends AutonomousBase {
    @Override
    public void runOpMode() {
        waitForStart();
        telemetry.addData("Run B", "successful");
        telemetry.update();
        sleep(1000);
    }
}
