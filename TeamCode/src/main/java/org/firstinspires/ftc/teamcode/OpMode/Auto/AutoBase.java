package org.firstinspires.ftc.teamcode.OpMode.Auto;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.Subsystem.AutoDriveSubsystem;

public abstract class AutoBase extends CommandOpMode {
    protected AutoDriveSubsystem autoDriveSubsystem;
    protected Follower follower;
    public abstract void makeAuto();
    public abstract void buildpaths();
}
