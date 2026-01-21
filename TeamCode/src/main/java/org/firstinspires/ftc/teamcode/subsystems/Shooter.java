package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;

@Configurable
public class Shooter extends SubsystemBase {
    public enum ShooterMode{
        RAW, PIDF,
        //FIXED,
    }

    private ShooterMode shooterMode = ShooterMode.RAW;

    //Raw Power Mode
    private double power = 0.0;
    public static double SHOOT_POWER = 0.1;

    //PIDF Mode
    private double targetVelcity = 0.0;
    private PIDFController flywheelPID = new PIDFController(0,0,0,0);

    public Shooter () {}

    public void setPower(){
        shooterMode = ShooterMode.RAW;
        power = p;
    }

    public void spinUp(){

    }


}
