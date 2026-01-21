package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class MecanumDrive extends SubsystemBase {

    private final Follower follower;

    public MecanumDrive(HardwareMap map, Pose startingPose) {
        follower = Constants.createFollower(map);
        follower.setStartingPose(startingPose);
    }

    @Override
    public void periodic() {
        follower.update();
    }

    public void drive(double forward, double strafe, double turn) {
        follower.setTeleOpDrive(forward, strafe, turn, false);
    }

    public Pose getPose() {
        return follower.getPose();
    }

    public void resetHeading(double heading) {
        follower.setPose(follower.getPose().setHeading(heading));
    }
}
