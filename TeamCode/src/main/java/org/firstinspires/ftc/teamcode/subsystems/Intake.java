package org.firstinspires.ftc.teamcode.subsystems;

import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Robot.FunnyRobot;

public class Intake extends SubsystemBase {
    public enum intakeMode {
        INGESTING,
        DISCARDING,
        OFF,
    }

    public static intakeMode IntakeMode = intakeMode.OFF;

    //control --->
    public static double INGESTING_MOTOR_SPEED = 1.0;
    public static double DISCARDING_MOTOR_SPEED = -0.1;

    public Intake(){}

    public void setMode(intakeMode newMode){
        IntakeMode = newMode;
    }

    public void ingest() {
        IntakeMode = IntakeMode.INGESTING;
    }

    public void discard() {
        IntakeMode = IntakeMode.DISCARDING;
    }

    public void stop() {
        IntakeMode = IntakeMode.OFF;
    }

    public void toggle() {
        if (IntakeMode == IntakeMode.OFF) {
            IntakeMode = IntakeMode.INGESTING;
        } else {
            IntakeMode = IntakeMode.OFF;
        }
    }


    @Override
    public void periodic(){
        FunnyRobot robot = FunnyRobot.get();

        switch (IntakeMode){
            case INGESTING:
                //robot.intakeLeft.setPower(INGESTING_MOTOR_SPEED);
                //robot.intakeRight.setPower(INGESTING_MOTOR_SPEED);
                break;

            case DISCARDING:
                //robot.intakeLeft.setPower(DISCARDING_MOTOR_SPEED);
                //robot.intakeRight.setPower(DISCARDING_MOTOR_SPEED);
                break;

            case OFF:
                //robot.intakeLeft.setPower(0);
                //robot.intakeRight.setPower(0);
                break;

            default:
                break;
        }
    }
}
