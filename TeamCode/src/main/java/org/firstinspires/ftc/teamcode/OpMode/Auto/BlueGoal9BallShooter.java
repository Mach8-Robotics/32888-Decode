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
import org.firstinspires.ftc.teamcode.Subsystem.LaunchSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.commands.AutoDriveCommand;
import org.firstinspires.ftc.teamcode.commands.AutoIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.LaunchCommand;
import org.firstinspires.ftc.teamcode.commands.RetractCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous( name= "Blue Goal 9 Ball SHOOTER", group ="Blue")
public class BlueGoal9BallShooter extends AutoBase{
    Command setPathTo15, setPathTo14, setPathTo13, setPathTo12, setPathTo11, setPathTo10, setPathTo9, setPathTo8, setPathTo7, setPathTo6, setPathTo6a, setPathTo6b, setPathTo6c, setPathTo5, setPathTo4, setPathTo3, setPathTo2, setPathTo1, setPathTo0;
    Path path15, path14, path13, path12, path11, path10, path9, path8, path7, path6, path6a, path6b, path6c, path5, path4, path3, path2, path1, path0;
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
        follower.setStartingPose(new Pose(24,122,Math.toRadians(144)));
        autoDriveSubsystem = new AutoDriveSubsystem(follower, telemetry, new Pose(22.5,124.5,Math.toRadians(144)));
        launchSubsystem = new LaunchSubsystem(rightCatapultMotor,leftCatapultMotor);
        intakeSubsystem = new IntakeSubsystem(intake);
    }

    @Override
    public void buildpaths() {
        path0 = new Path(new BezierCurve(new Pose(24.000, 122.000), new Pose(26.000, 120.000)));
        path0.setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(144));

        // Path 1 is the final resting place, jump from path0 to path 2 for multiple ball collection.
        path1 = new Path(new BezierCurve(new Pose(26.000, 120.000), new Pose(24.000, 90.000)));

        // Start here from path0 to collect multiple balls
        path2 = new Path(new BezierCurve(new Pose(26.000,120.000), new Pose(48.000, 96.000)));
        path2.setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180));
        path3 = new Path(new BezierCurve(new Pose(48.000, 96.000), new Pose(48.000, 84.000)));
        path3.setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180));
        path4 = new Path(new BezierCurve(new Pose(48.000, 84.000), new Pose(15.000, 84.000)));
        path4.setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180));
        path5 = new Path(new BezierCurve(new Pose(13.000, 84.000), new Pose(36.000, 84.000)));
        path5.setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(144));
        path6 = new Path(new BezierCurve(new Pose(36.000, 72.000), new Pose(36.000, 108.000)));
        path6.setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180));
        path6a = new Path(new BezierCurve(new Pose(36.000, 84.000), new Pose(36.000, 75.000)));
        path6a.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0));
        path6b = new Path(new BezierCurve(new Pose(36.000, 75.000), new Pose(18.000, 75.000)));
        path6b.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0));
        path6c = new Path(new BezierCurve(new Pose(18.000, 75.000), new Pose(36.000, 75.000)));
        path6c.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0));
        path7 = new Path(new BezierCurve(new Pose(36.000, 108.000), new Pose(26.000, 120.000)));
        path7.setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(144));
        path8 = new Path(new BezierCurve(new Pose(48.000, 96.000), new Pose(48.000, 60.000)));
        path8.setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180));
        path9 = new Path(new BezierCurve(new Pose(48.000, 60.000), new Pose(12.000, 60.000)));
        path9.setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180));
        path10 = new Path(new BezierCurve(new Pose(12.000, 60.000), new Pose(36.000, 60.000)));
        path10.setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180));
        path11= new Path(new BezierCurve(new Pose(36.000, 60.000), new Pose(36.000, 108.000)));
        path11.setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180));
        path12= new Path(new BezierCurve(new Pose(48.000, 96.000), new Pose(48.000, 36.000)));
        path12.setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180));
        path13= new Path(new BezierCurve(new Pose(48.000, 36.000), new Pose(12.000, 36.000)));
        path13.setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180));
        path14= new Path(new BezierCurve(new Pose(12.000, 36.000), new Pose(36.000, 36.000)));
        path14.setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(144));
        path15= new Path(new BezierCurve(new Pose(48.000, 96.000), new Pose(36.000, 72.000)));
        path15.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0));
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
        setPathTo6a = new InstantCommand(()->{
            autoDriveSubsystem.followPath(path6a, true);
        });
        setPathTo6b = new InstantCommand(()->{
            autoDriveSubsystem.followPath(path6b, true);
        });
        setPathTo6c = new InstantCommand(()->{
            autoDriveSubsystem.followPath(path6c, true);
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
                new InstantCommand(()->{follower.setMaxPower(0.6);}),
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
                setPathTo6a,
                new AutoDriveCommand(autoDriveSubsystem, telemetry).withTimeout(4000),
                setPathTo6b,
                new InstantCommand(()->{follower.setMaxPower(0.6);}),
                new AutoDriveCommand(autoDriveSubsystem, telemetry).withTimeout(4000),
                new WaitCommand(1000),
                setPathTo6c,
                new InstantCommand(()->{follower.setMaxPower(1);}),
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
                new InstantCommand(()->{follower.setMaxPower(0.6);}),
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
                setPathTo15,
                new AutoDriveCommand(autoDriveSubsystem, telemetry).withTimeout(4000)
               // setPathTo12,
              //  new AutoDriveCommand(autoDriveSubsystem, telemetry),
              //  setPathTo13,
                // Start intake
              //  new InstantCommand(()->{follower.setMaxPower(0.5);}),
                // Move but Slow
             //   new AutoDriveCommand(autoDriveSubsystem, telemetry),
                // Wait?
             //   setPathTo14,
                // Stop intake
              //  new InstantCommand(()->{follower.setMaxPower(1);}),
             //   new ParallelCommandGroup(new AutoDriveCommand(autoDriveSubsystem, telemetry), new RetractCommand(launchSubsystem)),

               // setPathTo15,
               // new AutoDriveCommand(autoDriveSubsystem, telemetry)
                // setPathTo7,
                // new AutoDriveCommand(autoDriveSubsystem, telemetry),
                //new RetractCommand(launchSubsystem),
                // new WaitCommand(500),
                // new LaunchCommand(launchSubsystem),
                //setPathTo1,
                // new AutoDriveCommand(autoDriveSubsystem, telemetry)
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
