package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

/*
 *
 * This is an example LinearOpMode that shows how to use
 * a Modern Robotics Color Sensor.
 *
 * The op mode assumes that the color sensor
 * is configured with a name of "sensor_color".
 *
 * You can use the X button on gamepad1 to toggle the LED on and off.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */
@TeleOp(name = "Color Sensor Tests", group = "Sensor")
@Disabled
public class SensorMRColor extends LinearOpMode {
    ColorSensor colorSensor;    // Hardware Device Object

    @Override
    public void runOpMode() {
        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F,0F,0F};

        // xPrevious represents the previous of the button.
        boolean xPrevious = false;

        // ledState represents the state of the LED.
        boolean ledState = true;

        // get a reference to our ColorSensor object.
        colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");

        // Set the LED in the beginning
        colorSensor.enableLed(true);

        // wait for the start button to be pressed.
        waitForStart();

        // while the op mode is active, loop and read the RGB data.
        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive()) {
            // toggle LED when x is pressed
            if (gamepad1.x && !xPrevious)  {
                ledState = !ledState;
                colorSensor.enableLed(ledState);
            }

            // update previous state variable.
            xPrevious = gamepad1.x;

            // convert the RGB values to HSV values.
            Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);

            // send the info back to driver station using telemetry function.
            telemetry.addData("LED  ", ledState ? "On" : "Off");
            telemetry.addData("Alpha", colorSensor.alpha());
            telemetry.addData("Red  ", colorSensor.red());
            telemetry.addData("Green", colorSensor.green());
            telemetry.addData("Blue ", colorSensor.blue());
            telemetry.addData("Hue  ", hsvValues[0]);
            telemetry.addData("Value", hsvValues[2]);
            telemetry.addData("Color", getColor(hsvValues));

            telemetry.update();
        }
    }

    // Returns either black green or yellow based on the hsv read (SUBJECT TO CHANGE)
    public String getColor(float[] hsv) {
        if (hsv[2] < 100) return "black";
        if (hsv[0] > 100) return "green";
        else return "yellow";
    }
}

