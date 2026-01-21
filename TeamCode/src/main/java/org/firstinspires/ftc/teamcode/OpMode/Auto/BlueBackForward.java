package org.firstinspires.ftc.teamcode.OpMode.Auto;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Subsystem.AutoDriveSubsystem;
import org.firstinspires.ftc.teamcode.commands.AutoDriveCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
@Autonomous
public class BlueBackForward extends AutoBase{
    Command setPathTo1;
    Path path1;
    @Override
    public void makeAuto() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(56,8,Math.toRadians(90)));
        autoDriveSubsystem = new AutoDriveSubsystem(follower, telemetry, new Pose(56,8,Math.toRadians(90)));
    }

    @Override
    public void buildpaths() {
        path1 = new Path(new BezierCurve(new Pose(56.000, 8.000), new Pose(48.000, 33.000)));
        path1.setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90));

    }

    @Override
    public void initialize() {
        makeAuto();
        buildpaths();
        register(autoDriveSubsystem);

        setPathTo1 = new InstantCommand(()->{
            autoDriveSubsystem.followPath(path1, true);
        });
        SequentialCommandGroup runAuto = new SequentialCommandGroup(
                setPathTo1,
                new AutoDriveCommand(autoDriveSubsystem, telemetry)
        );
        schedule(new SequentialCommandGroup(
                runAuto,
                new InstantCommand(()->{
                    follower.breakFollowing();
                    requestOpModeStop();
                })
        ));
    }
}
