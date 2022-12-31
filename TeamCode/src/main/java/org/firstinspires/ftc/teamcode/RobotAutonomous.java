package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="Autonomous Green", group="Linear Opmode")
public class RobotAutonomous extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    private DriveTrain motors = null;
    private String none;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        motors = new DriveTrain(hardwareMap);

        waitForStart();

        runtime.reset();
        motors.strafe(60);
        motors.drive(60);
        motors.linearMove(60 * Math.sqrt(2), 135);
        motors.drive(-120);
    }
}
