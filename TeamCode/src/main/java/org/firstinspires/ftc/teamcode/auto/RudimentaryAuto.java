package org.firstinspires.ftc.teamcode.auto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Autonomous(name="Funny Auto",group="Auto")
public class RudimentaryAuto extends LinearOpMode {
    private DcMotor frontLeftDrive;
    private DcMotor backLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backRightDrive;

    private ElapsedTime runTimer = new ElapsedTime();

    double number = 0;

    @Override
    public void runOpMode() {

        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        waitForStart();

        if (isStopRequested()) return;


        if (number < 1) {
            driveForward(50, 0.01);
            runTimer.reset();
            number += 1;
        }
        telemetry.addData("runTimer", runTimer.seconds());
        telemetry.update();
    }

    double timesRan = 0;

    private void driveForward(int ticks, double power) {

        double v = timesRan + 1;
        telemetry.addData("amounts this method has been ran", v);
        frontLeftDrive.setTargetPosition(frontLeftDrive.getCurrentPosition() + ticks);
            frontRightDrive.setTargetPosition(frontRightDrive.getCurrentPosition() + ticks);
            backLeftDrive.setTargetPosition(backLeftDrive.getCurrentPosition() + ticks);

            backRightDrive.setTargetPosition(backRightDrive.getCurrentPosition() + ticks);

            frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            frontLeftDrive.setPower(power);
            frontRightDrive.setPower(power);
            backLeftDrive.setPower(power);
            backRightDrive.setPower(power);

            while (opModeIsActive() && (
                    frontLeftDrive.isBusy() ||
                            frontRightDrive.isBusy() ||
                            backLeftDrive.isBusy() ||
                            backRightDrive.isBusy()
            )) {


                telemetry.update();
            }
            //stops after allotted time
            if (runTimer.seconds() >= 10) {
                frontLeftDrive.setPower(0);
                frontRightDrive.setPower(0);
                backLeftDrive.setPower(0);
                backRightDrive.setPower(0);
            }

    }
}