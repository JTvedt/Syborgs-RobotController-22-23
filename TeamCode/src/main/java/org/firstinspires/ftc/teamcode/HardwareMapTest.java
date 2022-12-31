package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;

/**
 * Test the motors to see if they are mapped correctly in the Configuration
 * Before pressing play, hold the robot upside down to see each wheel
 */
@Autonomous(name="Test Hardware Map", group="Linear Opmode")
public class HardwareMapTest extends LinearOpMode {
    public void runOpMode() {
        ArrayList<DcMotor> motorList = new ArrayList<DcMotor>();
        motorList.add(hardwareMap.get(DcMotor.class, "FL"));
        motorList.add(hardwareMap.get(DcMotor.class, "FR"));
        motorList.add(hardwareMap.get(DcMotor.class, "BL"));
        motorList.add(hardwareMap.get(DcMotor.class, "BR"));

        waitForStart();

        for (DcMotor motor : motorList) {
            motor.setPower(0.2);
            sleep(1000);
            motor.setPower(0.0);
            sleep(500);
        }
    }
}
