package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.Robot.FunnyRobot;

@Configurable
public class Shooter extends SubsystemBase {
    public enum ShooterMode{
        RAW, PIDF,
        //FIXED,
    }

    private ShooterMode shooterMode = ShooterMode.RAW;

    //Raw Power Mode
    private double power = 0.0;
    public static double SHOOT_POWER = 0.9;

    //PIDF Mode
    private double targetVelocity = 0.0;

    public static double kP = 300;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF = 10;
    private PIDFController flywheelPID = new PIDFController(kP,kI,kD,kF);

    public Shooter () {}

    public void setPower(double p){
        shooterMode = ShooterMode.RAW;
        power = p;
    }

    public void spinUp(){
        shooterMode = ShooterMode.RAW;
        power = SHOOT_POWER;
    }

    public void stop(){
        shooterMode = ShooterMode.RAW;
        power = 0.0;
    }

    public void setVelocity(double velocity){
        shooterMode = ShooterMode.PIDF;
        targetVelocity = velocity;
    }

    public void updatePIDF(double p, double i, double d, double f) {
        kP = p;
        kI = i;
        kD = d;
        kF = f;
        flywheelPID.setPIDF(kP, kI, kD, kF);
    }


    @Override
    public void periodic (){
        FunnyRobot robot = FunnyRobot.get();

        switch(shooterMode){

            case RAW:
                //robot.shooterLeftHood.setPower(power);
                //robot.shooterRightHood.setPower(power);
                break;

            case PIDF:
                double currentVelocity = robot.shooterLeftHood.getVelocity();
                double output = flywheelPID.calculate(currentVelocity, targetVelocity);

                //robot.shooterLeftHood.setPower(output);
                //robot.shooterRightHood.setPower(output);
                break;
        }
    }
}
