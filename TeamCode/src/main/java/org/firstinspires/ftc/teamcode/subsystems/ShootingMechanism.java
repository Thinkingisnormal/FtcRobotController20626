package org.firstinspires.ftc.teamcode.subsystems;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teleop.BasicOmniOpMode_Movement;

public class ShootingMechanism {
    final double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
    final double FULL_SPEED = 1.0;
    //for launcher encoders.
    final double LAUNCHER_TARGET_VELOCITY = 1125;
    final double LAUNCHER_MIN_VELOCITY = 1075;
    // Declare OpMode members
    //timers
    final double FEED_TIME_SECONDS = 0.2; //The feeder servos run this long when a shot is requested.
    final double TIME_BEFORE_LAUNCH = 1.0; //seconds before servos turn on to shoot when launcher hiits target velocity
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime feederTimer = new ElapsedTime();
    private ElapsedTime launcherTimer = new ElapsedTime();

    private DcMotorEx launcher;
    private CRServo leftFeeder;
    private CRServo rightFeeder;
    private enum LaunchState {IDLE, SPIN_UP, LAUNCH, LAUNCHING,}
    private LaunchState launchState;



}
