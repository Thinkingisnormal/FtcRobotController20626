package org.firstinspires.ftc.teamcode.mechanisms;


import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;
public class AprilTagLimelight  {
    private Limelight3A limelight;
    private IMU imu;

    private Telemetry telemetry;

    private double calcedDistance;

    public void LimeInit(HardwareMap hardwareMap, IMU imu, Telemetry telemetry) {
        this.limelight = hardwareMap.get(Limelight3A.class, "limelight");
        this.telemetry = telemetry;
        this.imu = imu;
        limelight.pipelineSwitch(0);
        limelight.start(); // swap to start() if you want to conserve battery.
    }

    public void Stop() {
        limelight.stop();
    }

    public void aprilTagLoop() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw()); // tells bot what our current yaw/heading is
        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            Pose3D botPose = llResult.getBotpose_MT2();
            telemetry.addData("Tx",llResult.getTx()); // target x
            telemetry.addData("Ty",llResult.getTy()); // target y
            telemetry.addData("Ta",llResult.getTa()); // area of target in pixels
            telemetry.addData("botpose", botPose.toString());
            calcedDistance = calcDistance(llResult.getTy());
            telemetry.addData("Distance (in):", calcedDistance);

        }
        else {
            telemetry.addLine("Apriltag is not in view.");
        }

        List<LLResultTypes.FiducialResult> fiducials = llResult.getFiducialResults();
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            int id = fiducial.getFiducialId(); // The ID number of the fiducial
            double x = fiducial.getTargetXDegrees(); // Where it is (left-right)
            double y = fiducial.getTargetYDegrees(); // Where it is (up-down)
            telemetry.addData("botpose",fiducial.getRobotPoseTargetSpace());
            calcDistance(llResult.getTy());
        }

    }
   public double calcDistance (double ty) {

        // how many degrees back is your limelight rotated from perfectly vertical?
        double limelightMountAngleDegrees = 0;

        // distance from the center of the Limelight lens to the floor
        double limelightLensHeightInches = 11.5;

        // distance from the target to the floor
        double goalHeightInches = 28.5;

        double angleToGoalDegrees = limelightMountAngleDegrees + ty;
        double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

        //calculate distance
        double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
        telemetry.addData("Distance:",distanceFromLimelightToGoalInches);
        return distanceFromLimelightToGoalInches;
    }


    double MIN_DISTANCE = 59; //distance is in inches
    double MAX_DISTANCE = 61; //distance is in inches

    //rumbles gamepad if distances is in range of min and max to notify the driver they are in good spot.
    public void rumbleForDistance (Gamepad gamepad) {
        if ((calcedDistance > MIN_DISTANCE) && (calcedDistance < MAX_DISTANCE)) {
            gamepad.rumble(200);
        } else {gamepad.stopRumble();}
    }

}
