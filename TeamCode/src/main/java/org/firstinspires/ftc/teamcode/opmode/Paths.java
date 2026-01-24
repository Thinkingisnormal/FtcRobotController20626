package org.firstinspires.ftc.teamcode.opmode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public  class Paths {
    public PathChain Path1;
    public PathChain Path2;
    public PathChain Path3;
    public PathChain Path4;
    public PathChain Path5;
    public PathChain Path6;
    public PathChain Path7;

    public Paths(Follower follower) {
        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(20.464, 127.529),

                                new Pose(52.007, 98.392)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(52.007, 98.392),

                                new Pose(50.610, 83.045)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(50.610, 83.045),

                                new Pose(17.718, 83.000)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        Path4 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(17.718, 83.000),

                                new Pose(51.901, 98.389)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        Path5 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(51.901, 98.389),

                                new Pose(46.367, 59.529)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(130))

                .build();

        Path6 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(46.367, 59.529),

                                new Pose(16.005, 59.406)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        Path7 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(16.005, 59.406),

                                new Pose(51.910, 98.329)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(130))

                .build();
    }
}
/*public static class Paths {
    public PathChain Path1;
    public PathChain Path2;
    public PathChain Path3;
    public PathChain Path4;
    public PathChain Path5;
    public PathChain Path6;
    public PathChain Path7;
    public PathChain Path8;

    public Paths(Follower follower) {
        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(20.464, 127.529),

                                new Pose(52.007, 98.392)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed(true)
                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(52.007, 98.392),

                                new Pose(50.610, 83.045)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(50.610, 83.045),

                                new Pose(17.718, 83.000)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        Path4 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(17.718, 83.000),

                                new Pose(51.901, 98.389)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        Path5 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(51.901, 98.389),

                                new Pose(46.367, 59.529)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(130))

                .build();

        Path6 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(46.367, 59.529),

                                new Pose(16.005, 59.406)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        Path7 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(16.005, 59.406),

                                new Pose(51.910, 98.329)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(130))

                .build();

        Path8 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(51.910, 98.329),

                                new Pose(101.073, 36.785)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();


          [(20.464, 127.529), (52.007, 98.392), (50.610, 83.045), (17.718, 83.000),
         (51.901, 98.389), (46.367, 59.529), (16.005, 59.406), (51.910, 98.329), (101.073, 36.785)]
    }
}*/