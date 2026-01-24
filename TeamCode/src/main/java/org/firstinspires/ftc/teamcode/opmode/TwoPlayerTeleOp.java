package org.firstinspires.ftc.teamcode.opmode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.FunnyRobot;

@TeleOp(name="Two Player Teleop", group="opmodes")
@Configurable
public class TwoPlayerTeleOp extends OpMode {
    private FunnyRobot robot;
    private GamepadEx gamepad1Ex;
    private GamepadEx gamepad2Ex;


    @Override
    public void init(){
        //robot = FunnyRobot.get().init();
        gamepad1Ex = new GamepadEx(gamepad1);
        gamepad2Ex = new GamepadEx(gamepad2);

    }

    @Override
    public void loop(){



    }
}
