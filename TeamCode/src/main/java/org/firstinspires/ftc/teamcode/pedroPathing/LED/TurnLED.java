package org.firstinspires.ftc.teamcode.pedroPathing.LED;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp(name = "Turn LED", group = "TeleOp")
public class TurnLED extends OpMode {

    LEDStest bench = new LEDStest();

    @Override
    public void init() {
        bench.init(hardwareMap);


    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            bench.setRedLed(true);
            bench.setGreenLed(false);
            bench.setBlueLed(false);

        } else if (gamepad1.b) {
            bench.setRedLed(false);
            bench.setGreenLed(true);
            bench.setBlueLed(false);
        } else if (gamepad1.y) {
            bench.setRedLed(false);
            bench.setGreenLed(false);
            bench.setBlueLed(true);

        }
    }
}

