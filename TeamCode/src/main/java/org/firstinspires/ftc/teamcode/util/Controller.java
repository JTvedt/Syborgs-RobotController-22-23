package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.HashMap;

public class Controller {
    Gamepad gamepad1;
    Gamepad gamepad2;

    HashMap<String, Button> map1 = new HashMap<>();
    HashMap<String, Button> map2 = new HashMap<>();

    static final String[] buttonKeys = {
            "A", "B", "X", "Y",
            "DU", "DD", "DL", "DR",
            "LB", "RB", "LS", "RS"
    };

    static final String[] buttonList = {
            "a", "b", "x", "y",
            "dpad_up", "dpad_down",
            "dpad_left", "dpad_right",
            "left_bumper", "right_bumper",
            "left stick button", "right stick button"
    };

    boolean singleplayer = false;

    public Controller(Gamepad gamepad1, Gamepad gamepad2) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;

        for (int i = 0; i < buttonKeys.length; i++) {
            map1.put(buttonKeys[i], new Button(buttonList[i]));
            map2.put(buttonKeys[i], new Button(buttonList[i]));
        }
    }

    public Controller(Gamepad gamepad1) {
        this(gamepad1, null);
        singleplayer = true;
    }

    public void update() {
        String gamepad1Read = gamepad1.toString();
        for (String key : buttonKeys) {
            map1.get(key).update(gamepad1Read);
        }

        if (singleplayer) return;

        String gamepad2Read = gamepad2.toString();
        for (String key : buttonKeys) {
            map2.get(key).update(gamepad2Read);
        }
    }

    public boolean hold(int controller, String button) {
        if (controller == 1) return map1.get(button).hold;
        else return map2.get(button).hold;
    }

    public boolean hold(String button) {
        return hold(1, button);
    }

    public boolean press(int controller, String button) {
        if (controller == 1) return map1.get(button).pressing();
        else return map2.get(button).pressing();
    }

    public boolean press(String button) {
        return press(1, button);
    }

    static class Button {
        public Gamepad gamepad;
        public String button;

        public boolean press = false;
        public boolean hold = false;

        public Button(String button) {
            this.button = button;
        }

        public void update(String gamepadRead) {
            boolean state = gamepadRead.contains(" " + this.button + " ");

            if (state && !hold) {
                hold = true;
                press = true;
            } else if (!state && hold) {
                hold = false;
                press = false;
            }
        }

        public boolean pressing() {
            if (press) {
                press = false;
                return true;
            }
            return false;
        }
    }
}
