package org.firstinspires.ftc.teamcode.oldcode.zoldcode.oldauto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous (name="funny auto 2 (it better work)", group="Auto")
public class ShootingAuto extends OpMode{

    final double STOP_SPEED = 0.0;
    final double FULL_SPEED = 1.0;

    final double LAUNCHER_TARGET_VELOCITY = 1125;
    final double LAUNCHER_MIN_VELOCITY = 1075;

    final double FEED_TIME_SECONDS = 0.2;
    final double TIME_BEFORE_LAUNCH = 1.0;

    // === HARDWARE ===
    private DcMotorEx launcher;
    private CRServo leftFeeder;
    private CRServo rightFeeder;

    // === TIMERS ===
    private ElapsedTime spinupTimer = new ElapsedTime();
    private ElapsedTime feedTimer = new ElapsedTime();

    private enum AutoState {IDLE,SPINUP, FEED, DONE}
    private AutoState state;

    private int shotcount = 0;

    private HardwareMap hwMap;

    @Override
    public void init(){
        launcher = hwMap.get(DcMotorEx.class, "launcher");
        leftFeeder = hwMap.get(CRServo.class,"left_feeder");
        rightFeeder = hwMap.get(CRServo.class,"right_feeder");

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(300, 0, 0, 10));

        leftFeeder.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFeeder.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFeeder.setPower(STOP_SPEED);
        rightFeeder.setPower(STOP_SPEED);

        state = AutoState.IDLE;

        telemetry.addLine("Initialized.");



    }
    @Override
    public void start (){
        state = AutoState.SPINUP;
        spinupTimer.reset();
        launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);

    }

    public void loop (){
            switch (state) {

                case SPINUP:
                    if (launcher.getVelocity() >= LAUNCHER_MIN_VELOCITY &&
                            spinupTimer.seconds() > TIME_BEFORE_LAUNCH) {

                        state = AutoState.FEED;
                        feedTimer.reset();

                        leftFeeder.setPower(FULL_SPEED);
                        rightFeeder.setPower(FULL_SPEED);
                    }
                    break;

                case FEED:
                    if (feedTimer.seconds() > FEED_TIME_SECONDS) {

                        // stop feeders
                        leftFeeder.setPower(STOP_SPEED);
                        rightFeeder.setPower(STOP_SPEED);

                        shotcount++;

                        if (shotcount >= 3) {
                            launcher.setVelocity(0);
                            state = AutoState.DONE;
                        } else {
                            // Spin up again for next shot
                            state = AutoState.SPINUP;
                            spinupTimer.reset();
                        }
                    }
                    break;

                case DONE:
                    // Auto finished
                    break;
            }

            telemetry.addData("State", state);
            telemetry.addData("Shot Fired", shotcount);
            telemetry.addData("Launcher Velocity", launcher.getVelocity());
        }
    }