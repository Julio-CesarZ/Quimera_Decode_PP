package org.firstinspires.ftc.teamcode.pedroPathing.LED;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;

public class LEDStest {

    private LED RedLed;
    private LED GreenLed;
    private LED BlueLed;

    public LED getRedLed() {
        return RedLed;
    }
    public void init(HardwareMap hwMap){
        RedLed = hwMap.get(LED.class, "RED");
        GreenLed = hwMap.get(LED.class, "GREEN");
        BlueLed = hwMap.get(LED.class, "BLUE");
    }

    public void setRedLed(Boolean is0n) {
        if(is0n) {
            RedLed.on();
        } else {
            RedLed.off();
        }

    }

    public void setGreenLed(Boolean is0n) {
        if(is0n) {
            GreenLed.on();
        } else {
            GreenLed.on();
        }

    }
    public void setBlueLed(Boolean is0n) {
        if(is0n) {
            BlueLed.on();
        } else {
            BlueLed.on();
        }

    }
}
