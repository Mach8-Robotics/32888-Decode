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
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class BlueGoal extends AutoBase{
    Command setPathTo1;
    Path path1;
    private DcMotor rightCatapultMotor, leftCatapultMotor;
    private LaunchSubsystem launchSubsystem;
    @Override
    public void makeAuto() {
        follower = Constants.createFollower(hardwareMap);
        rightCatapultMotor = hardwareMap.get(DcMotor.class,"catapult1");
        leftCatapultMotor = hardwareMap.get(DcMotor.class, "catapult2");
        follower.setStartingPose(new Pose(22.5,124.5,Math.toRadians(144)));
        autoDriveSubsystem = new AutoDriveSubsystem(follower, telemetry, new Pose(22.5,124.5,Math.toRadians(144)));
        launchSubsystem = new LaunchSubsystem(rightCatapultMotor,leftCatapultMotor);
    }

    @Override
    public void buildpaths() {
        path1 = new Path(new BezierCurve(new Pose(22.5, 124.5), new Pose(18.000, 102.000)));
        path1.setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180));

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
