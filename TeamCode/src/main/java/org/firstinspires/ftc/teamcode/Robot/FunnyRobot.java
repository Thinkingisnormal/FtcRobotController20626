package org.firstinspires.ftc.teamcode.Robot;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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
    public MotorEx shooterRightHood, shooterLeftHood;
    public ServoEx boosterRight, boosterLeft;

    //Telemetry
    public Telemetry t;

    private MotorEx frontRight, frontLeft, backRight, backLeft;

    //Sensors
    public Limelight3A limelight;

    public HardwareMap hwMap;

    //Subsystems
    public MecanumDrive drive;
    public Shooter shooter;
    public Intake intake;
    public RobotEyes vision;
    public Booster booster;

    /*
    For later...
    private Pose teleOpStart = new Pose(0, 0, 0);
    private Pose autoStart   = new Pose(0, 0, 0);
    */

    public static FunnyRobot get(){
        return inst;
    }

    public FunnyRobot(){}

    public void init(HardwareMap hwMap, Opmode mode, Telemetry t){
        this.hwMap = hwMap;
        this.t = t;

        //Mecanum Wheels
        frontLeft  = new MotorEx(hwMap, "front_left_drive");
        frontRight = new MotorEx(hwMap, "front_right_drive");
        backLeft   = new MotorEx(hwMap, "back_left_drive");
        backRight  = new MotorEx(hwMap, "back_right_drive");

        //Intake Motors
        intakeLeft  = new MotorEx(hwMap, "intake_left");
        intakeRight = new MotorEx(hwMap, "intake_right");

        shooterLeftHood  = new MotorEx(hwMap, "shooter_left");
        shooterRightHood = new MotorEx(hwMap, "shooter_right");

        boosterLeft  = new ServoEx(hwMap, "booster_left");
        boosterRight = new ServoEx(hwMap, "booster_right");


    }

}
