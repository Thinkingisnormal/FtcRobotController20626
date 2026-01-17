package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;


public class DigitalLEDIndicator {

    private LED redLED;
    private LED greenLED;

    public void init (HardwareMap hardwareMap) {
        redLED = hardwareMap.get(LED.class, "led_red");
        greenLED = hardwareMap.get(LED.class, "led_green");
    }

    public void setRED (boolean isOn) {
        if (isOn) { redLED.on();}
        else { redLED.off();}
    }

    public void setGREEN (boolean isOn) {
        if (isOn) { greenLED.on();}
        else { greenLED.off();}
    }
}
