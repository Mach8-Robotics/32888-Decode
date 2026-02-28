package org.firstinspires.ftc.teamcode.OpMode.Auto;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Subsystem.AutoDriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.LaunchSubsystem;
import org.firstinspires.ftc.teamcode.commands.AutoDriveCommand;
import org.firstinspires.ftc.teamcode.commands.AutoIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.LaunchCommand;
import org.firstinspires.ftc.teamcode.commands.RetractCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous( name= "Red Goal 12 Balls", group ="Red")
public class RedGoal12Balls extends AutoBase{
    Command setPathTo15, setPathTo14, setPathTo13, setPathTo12, setPathTo11, setPathTo10, setPathTo9, setPathTo8, setPathTo7, setPathTo6, setPathTo5, setPathTo4, setPathTo3, setPathTo2, setPathTo1, setPathTo0;
    Path path15, path14, path13, path12, path11, path10, path9, path8, path7, path6, path5, path4, path3, path2, path1, path0;
    private DcMotor rightCatapultMotor, leftCatapultMotor, intake;
    private LaunchSubsystem launchSubsystem;
     private IntakeSubsystem intakeSubsystem;
    @Override
    public void makeAuto() {
        follower = Constants.createFollower(hardwareMap);
        rightCatapultMotor = hardwareMap.get(DcMotor.class,"catapult1");
        leftCatapultMotor = hardwareMap.get(DcMotor.class, "catapult2");
        intake = hardwareMap.get(DcMotor.class, "intake");

        // intakeMotor = hardwareMap.get(DcMotor.class,"intake");
        // Check this
        follower.setStartingPose(new Pose(24,122,Math.toRadians(144)).mirror());
        autoDriveSubsystem = new AutoDriveSubsystem(follower, telemetry, new Pose(22.5,124.5,Math.toRadians(144)).mirror());
        launchSubsystem = new LaunchSubsystem(rightCatapultMotor,leftCatapultMotor);
        intakeSubsystem = new IntakeSubsystem(intake);
    }

    @Override
    public void buildpaths() {
        path0 = new Path(new BezierCurve(new Pose(24.000, 122.000).mirror(), new Pose(26.000, 120.000).mirror()));
        path0.setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(36));

        // Path 1 is the final resting place, jump from path0 to path 2 for multiple ball collection.
        path1 = new Path(new BezierCurve(new Pose(26.000, 120.000).mirror(), new Pose(24.000, 90.000).mirror()));

        // Start here from path0 to collect multiple balls
        path2 = new Path(new BezierCurve(new Pose(26.000,120.000).mirror(), new Pose(48.000, 96.000).mirror()));
        path2.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0));
        path3 = new Path(new BezierCurve(new Pose(48.000, 96.000).mirror(), new Pose(48.000, 84.000).mirror()));
        path3.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0));
        path4 = new Path(new BezierCurve(new Pose(48.000, 84.000).mirror(), new Pose(15.000, 84.000).mirror()));
        path4.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0));
        path5 = new Path(new BezierCurve(new Pose(13.000, 84.000).mirror(), new Pose(36.000, 84.000).mirror()));
        path5.setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(36));
        path6 = new Path(new BezierCurve(new Pose(36.000, 84.000).mirror(), new Pose(36.000, 108.000).mirror()));
        path6.setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(36));
        path7 = new Path(new BezierCurve(new Pose(36.000, 108.000).mirror(), new Pose(26.000, 120.000).mirror()));
        path7.setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(36));
        path8 = new Path(new BezierCurve(new Pose(48.000, 96.000).mirror(), new Pose(48.000, 60.000).mirror()));
        path8.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0));
        path9 = new Path(new BezierCurve(new Pose(48.000, 60.000).mirror(), new Pose(12.000, 60.000).mirror()));
        path9.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0));
        path10 = new Path(new BezierCurve(new Pose(12.000, 60.000).mirror(), new Pose(36.000, 60.000).mirror()));
        path10.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0));
        path11= new Path(new BezierCurve(new Pose(36.000, 60.000).mirror(), new Pose(36.000, 108.000).mirror()));
        path11.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0));
        path12= new Path(new BezierCurve(new Pose(48.000, 96.000).mirror(), new Pose(48.000, 36.000).mirror()));
        path12.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0));
        path13= new Path(new BezierCurve(new Pose(48.000, 36.000).mirror(), new Pose(12.000, 36.000).mirror()));
        path13.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0));
        path14= new Path(new BezierCurve(new Pose(12.000, 36.000).mirror(), new Pose(36.000, 36.000).mirror()));
        path14.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(36));
        path15= new Path(new BezierCurve(new Pose(36.000, 36.000).mirror(), new Pose(36.000, 108.000).mirror()));
        path15.setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(36));
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
        setPathTo2 = new InstantCommand(()->{
            autoDriveSubsystem.followPath(path2, true);
        });
        setPathTo3 = new InstantCommand(()->{
            autoDriveSubsystem.followPath(path3, true);
        });
        setPathTo4 = new InstantCommand(()->{
            autoDriveSubsystem.followPath(path4, true);
        });
        setPathTo5 = new InstantCommand(()->{
            autoDriveSubsystem.followPath(path5, true);
        });
        setPathTo6 = new InstantCommand(()->{
            autoDriveSubsystem.followPath(path6, true);
        });
        setPathTo7 = new InstantCommand(()->{
            autoDriveSubsystem.followPath(path7, true);
        });
        setPathTo8 = new InstantCommand(()->{
            autoDriveSubsystem.followPath(path8, true);
        });
        setPathTo9 = new InstantCommand(()->{
            autoDriveSubsystem.followPath(path9, true);
        });
        setPathTo10 = new InstantCommand(()->{
            autoDriveSubsystem.followPath(path10, true);
        });
        setPathTo11 = new InstantCommand(()->{
            autoDriveSubsystem.followPath(path11, true);
        });
        setPathTo12 = new InstantCommand(()->{
            autoDriveSubsystem.followPath(path12, true);
        });
        setPathTo13 = new InstantCommand(()->{
            autoDriveSubsystem.followPath(path13, true);
        });
        setPathTo14 = new InstantCommand(()->{
            autoDriveSubsystem.followPath(path14, true);
        });
        setPathTo15 = new InstantCommand(()->{
            autoDriveSubsystem.followPath(path15, true);
        });
        SequentialCommandGroup runAuto = new SequentialCommandGroup(
                setPathTo0,
                new AutoDriveCommand(autoDriveSubsystem, telemetry).withTimeout(3500),
                new RetractCommand(launchSubsystem),
                new WaitCommand(500),
                new LaunchCommand(launchSubsystem),
                new WaitCommand(100),
                new RetractCommand(launchSubsystem),
                setPathTo2,
                new AutoDriveCommand(autoDriveSubsystem, telemetry).withTimeout(3500),
                setPathTo3,
                new AutoDriveCommand(autoDriveSubsystem, telemetry).withTimeout(3500),
                setPathTo4,
                new InstantCommand(()->{follower.setMaxPower(0.5);}),
                new ParallelDeadlineGroup(
                        new AutoDriveCommand(autoDriveSubsystem, telemetry).withTimeout(4000),
                        new AutoIntakeCommand(intakeSubsystem)
                ),
                // Move but Slow

                // Wait?
                setPathTo5,
                new InstantCommand(()->{follower.setMaxPower(1);}),
                // Stop intake
                new AutoDriveCommand(autoDriveSubsystem, telemetry).withTimeout(4000),
                setPathTo6,

                new AutoDriveCommand(autoDriveSubsystem, telemetry).withTimeout(4000),
                new WaitCommand(250),

                setPathTo7,
                new AutoDriveCommand(autoDriveSubsystem, telemetry).withTimeout(4000),
                new LaunchCommand(launchSubsystem),
                new RetractCommand(launchSubsystem),
                setPathTo2,
                new AutoDriveCommand(autoDriveSubsystem, telemetry).withTimeout(4000),
                setPathTo8,
                new AutoDriveCommand(autoDriveSubsystem, telemetry).withTimeout(4000),
                setPathTo9,
                // Start intake
                new InstantCommand(()->{follower.setMaxPower(0.5);}),
                // Move but Slow
                new AutoDriveCommand(autoDriveSubsystem, telemetry).withTimeout(4000),
                // Wait?
                setPathTo10,
                // Stop intake
                new InstantCommand(()->{follower.setMaxPower(1);}),
                new AutoDriveCommand(autoDriveSubsystem, telemetry).withTimeout(4000),
                setPathTo11,
                new AutoDriveCommand(autoDriveSubsystem, telemetry).withTimeout(4000),
                setPathTo7,
                new AutoDriveCommand(autoDriveSubsystem, telemetry).withTimeout(4000),
                new WaitCommand(250),
                new LaunchCommand(launchSubsystem),
                new WaitCommand(250),
                new RetractCommand(launchSubsystem),
                setPathTo2,
                new AutoDriveCommand(autoDriveSubsystem, telemetry).withTimeout(4000),
                setPathTo12,
                new AutoDriveCommand(autoDriveSubsystem, telemetry).withTimeout(4000),
                setPathTo13,
                // Start intake
                new InstantCommand(()->{follower.setMaxPower(0.5);}),
                // Move but Slow
                new AutoDriveCommand(autoDriveSubsystem, telemetry).withTimeout(4000),
                // Wait?
                setPathTo14,
                // Stop intake
                new InstantCommand(()->{follower.setMaxPower(1);}),
                new ParallelCommandGroup(new AutoDriveCommand(autoDriveSubsystem, telemetry).withTimeout(4000), new RetractCommand(launchSubsystem)),

                setPathTo15,
                new AutoDriveCommand(autoDriveSubsystem, telemetry).withTimeout(4000),
                setPathTo7,
                new AutoDriveCommand(autoDriveSubsystem, telemetry).withTimeout(4000),
                new RetractCommand(launchSubsystem),
                new WaitCommand(500),
                new LaunchCommand(launchSubsystem),
                setPathTo1,
                new AutoDriveCommand(autoDriveSubsystem, telemetry).withTimeout(4000)
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
