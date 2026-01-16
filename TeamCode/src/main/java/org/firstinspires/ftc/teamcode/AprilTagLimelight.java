package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;

public class AprilTagLimelight extends OpMode {
    private Limelight3A limelight;
    private IMU imu;
    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start(); // swap to start() if you want to conserve battery.
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot =
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, //DIRECTIONS NOT CHECKED
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD); //DIRECTIONS NOT CHECKED
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
    }

    @Override
    public void start() {
        // swap to start() if you want to conserve battery.
    }


    @Override
    public void loop() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw()); // tells bot what our current yaw/heading is
        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            Pose3D botPose = llResult.getBotpose_MT2();
            telemetry.addData("Tx",llResult.getTx()); // target x
            telemetry.addData("Ty",llResult.getTy()); // target y
            telemetry.addData("Ta",llResult.getTa()); // area of target in pixels
            telemetry.addData("botpose", botPose.toString());

        }

        List<LLResultTypes.FiducialResult> fiducials = llResult.getFiducialResults();
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            int id = fiducial.getFiducialId(); // The ID number of the fiducial
            double x = fiducial.getTargetXDegrees(); // Where it is (left-right)
            double y = fiducial.getTargetYDegrees(); // Where it is (up-down)
            double distance = fiducial.getRobotPoseTargetSpace().getPosition().y; //idk if this works
            telemetry.addData("Fiducial " + id, "is " + distance + " meters away");
        }
    }
}
