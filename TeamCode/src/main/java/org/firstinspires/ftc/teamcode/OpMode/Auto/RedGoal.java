package org.firstinspires.ftc.teamcode.OpMode.Auto;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Subsystem.AutoDriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.LaunchSubsystem;
import org.firstinspires.ftc.teamcode.commands.AutoDriveCommand;
import org.firstinspires.ftc.teamcode.commands.LaunchCommand;
import org.firstinspires.ftc.teamcode.commands.RetractCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous( name= "Red Goal", group ="Red")
public class RedGoal extends AutoBase{
    Command setPathTo1, setPathTo0;
    Path path1, path0;
    private DcMotor rightCatapultMotor, leftCatapultMotor;
    private LaunchSubsystem launchSubsystem;
    @Override
    public void makeAuto() {
        follower = Constants.createFollower(hardwareMap);
        rightCatapultMotor = hardwareMap.get(DcMotor.class,"catapult1");
        leftCatapultMotor = hardwareMap.get(DcMotor.class, "catapult2");
        follower.setStartingPose(new Pose(24,122,Math.toRadians(144)).mirror());
        autoDriveSubsystem = new AutoDriveSubsystem(follower, telemetry, new Pose(22.5,124.5,Math.toRadians(144)).mirror());
        launchSubsystem = new LaunchSubsystem(rightCatapultMotor,leftCatapultMotor);
    }

    @Override
    public void buildpaths() {
        path0 = new Path(new BezierCurve(new Pose(24, 122).mirror(), new Pose(26, 120).mirror()));
        path0.setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(36));
        path1 = new Path(new BezierCurve(new Pose(26, 120).mirror(), new Pose(19.000, 108.000).mirror()));
        path1.setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(0));

    }

    @Override
    public void initialize() {
        makeAuto();
        buildpaths();
        register(autoDriveSubsystem);

        setPathTo0 = new InstantCommand(()->{
            autoDriveSubsystem.followPath(path0, true);
        });
        setPathTo1 = new InstantCommand(()->{
            autoDriveSubsystem.followPath(path1, true);
        });
        SequentialCommandGroup runAuto = new SequentialCommandGroup(
                setPathTo0,
                new AutoDriveCommand(autoDriveSubsystem, telemetry),
                setPathTo1,
                new RetractCommand(launchSubsystem),
                new LaunchCommand(launchSubsystem),
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
