package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.Robot.FunnyRobot;

@Configurable
public class Shooter extends SubsystemBase {
    public enum ShooterMode{
        RAW, PIDF, OFF,
    }

    private ShooterMode shooterMode = ShooterMode.RAW;

    //Raw Power Mode
    private double power = 0.0;
    public static double SHOOT_POWER = 0.9;

    //PIDF Mode
    private double targetVelocity = 5000;

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

    public void rawMode(){
        shooterMode = ShooterMode.RAW;
        power = SHOOT_POWER;
    }

    public void stop(){
        shooterMode = ShooterMode.RAW;
        power = 0.0;
    }

    public void OFFMODE(){
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

    public void setMode(ShooterMode shooterMode){
        this.shooterMode = shooterMode;
    }


    @Override
    public void periodic (){
        FunnyRobot robot = FunnyRobot.get();

        switch(shooterMode){

            case RAW:
                robot.shooterLeftHood.set(power);
                robot.shooterRightHood.set(power);
                break;

            case PIDF:
                double currentVelocity = robot.shooterLeftHood.getVelocity();
                double output = flywheelPID.calculate(currentVelocity, targetVelocity);

                robot.shooterLeftHood.set(output);
                robot.shooterRightHood.set(output);
                break;
            case OFF:

                break;
            default:
                break;
        }
    }
}
/*
package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import org.firstinspires.ftc.teamcode.Robot.FunnyRobot;

@Configurable
public class Shooter extends SubsystemBase {

    public enum ShooterMode {
        RAW,
        PIDF,
        OFF
    }

    private ShooterMode mode = ShooterMode.OFF;

    // RAW mode
    private double rawPower = 0.0;
    public static double RAW_SHOOT_POWER = 0.9;

    // PIDF mode
    public static double kP = 300;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF = 10;

    public static double TARGET_RPM = 3000; // tune this
    private double targetVelocity = 0;

    private final PIDFController controller = new PIDFController(kP, kI, kD, kF);

    public Shooter() {}

    // ===== RAW =====
    public void setRaw(double p) {
        mode = ShooterMode.RAW;
        rawPower = p;
    }

    public void spinUpRaw() {
        mode = ShooterMode.RAW;
        rawPower = RAW_SHOOT_POWER;
    }

    // ===== PIDF =====
    public void setVelocity(double velocity) {
        mode = ShooterMode.PIDF;
        targetVelocity = velocity;
    }

    public void spinUpPIDF() {
        mode = ShooterMode.PIDF;
        targetVelocity = TARGET_RPM;
    }

    public void updatePIDF(double p, double i, double d, double f) {
        kP = p;
        kI = i;
        kD = d;
        kF = f;
        controller.setPIDF(p, i, d, f);
    }

    // ===== OFF =====
    public void stop() {
        mode = ShooterMode.OFF;
        rawPower = 0;
        targetVelocity = 0;
    }

    @Override
    public void periodic() {
        FunnyRobot robot = FunnyRobot.get();

        double output = 0;

        switch (mode) {

            case RAW:
                output = rawPower;
                break;

            case PIDF:
                double currentVelocity = robot.shooterLeftHood.getVelocity();
                output = controller.calculate(currentVelocity, targetVelocity);
                break;

            case OFF:
                output = 0;
                break;
        }

        // clamp output
        output = Math.max(0, Math.min(1, output));

        robot.shooterLeftHood.set(output);
        robot.shooterRightHood.set(output);
    }
}

 */