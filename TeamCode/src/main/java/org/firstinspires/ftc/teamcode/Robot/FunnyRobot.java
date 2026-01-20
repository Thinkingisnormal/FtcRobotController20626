package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.teamcode.subsystems.Booster;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.RobotEyes;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

public class FunnyRobot {
    public static final FunnyRobot inst = new FunnyRobot();

    public enum Opmode {
        AUTO, TELEOP
    }
    public enum Alliance {
        RED, BlUE
    }

    //Alliance + Opmode settings
    public static Alliance alliance = Alliance.BlUE;
    public static Opmode mode = Opmode.AUTO;

    //hardware
    public MotorEx intakeRight, intakeLeft;
    public MotorEx shooterRightHoodHood, shooterLeftHood;
    public ServoEx boosterRight, boosterLeft;

    //Sensors
    public Limelight3A limelight;

    public HardwareMap hwMap;

    //Subsystems
    public MecanumDrive drive;
    public Shooter shooter;
    public Intake intake;
    public RobotEyes vision;
    public Booster booster;

    public static FunnyRobot get(){
        return inst;
    }

    public FunnyRobot(){}

}
