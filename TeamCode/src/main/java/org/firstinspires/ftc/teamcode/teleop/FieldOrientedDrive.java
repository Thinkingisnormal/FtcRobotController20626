package org.firstinspires.ftc.teamcode.teleop;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;

@TeleOp (name="Field Centric Drive",group = "Opmodes")
public class FieldOrientedDrive extends OpMode {
      // Note: pushing stick forward gives negative value
    MecanumDrive drive = new MecanumDrive();

    @Override
    public void init (){
        drive.init(hardwareMap);

    }

    @Override
    public void loop (){
       double  axial = -gamepad1.left_stick_y;

       double lateral = gamepad1.left_stick_x;

        double yaw = -gamepad1.right_stick_x;

        drive.fieldRelativeDrive(axial, lateral, yaw);


    }
}
