package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystem.AutoDriveSubsystem;

import java.util.function.DoubleSupplier;

public class AutoDriveCommand extends CommandBase {
    private AutoDriveSubsystem autoDriveSubsystem;
    private DoubleSupplier strafe,forward, turn;
    private boolean teleop;
    private Telemetry telemetry;
    public AutoDriveCommand(AutoDriveSubsystem autoDriveSubsystem, Telemetry telemetry){
        this.autoDriveSubsystem = autoDriveSubsystem;
        this.telemetry = telemetry;
        teleop = false;
        addRequirements(autoDriveSubsystem);
    }

    public AutoDriveCommand(AutoDriveSubsystem autoDriveSubsystem,
                            DoubleSupplier strafe,
                            DoubleSupplier forward,
                            DoubleSupplier turn,
                            Telemetry telemetry){
        this.autoDriveSubsystem = autoDriveSubsystem;
        this.telemetry = telemetry;
        teleop = true;
        this.strafe = strafe;
        this.turn = turn;
        this.forward = forward;
        addRequirements(autoDriveSubsystem);
    }

    @Override
    public void execute(){
        autoDriveSubsystem.update();
        if(teleop) {
            autoDriveSubsystem.setTeleOpMovementVectors(forward.getAsDouble(),
                    strafe.getAsDouble(),
                    turn.getAsDouble(),
                    true);
        }
    }
    @Override
    public boolean isFinished(){
        if(!teleop) {
            return !autoDriveSubsystem.isBusy();
        }
        else {
            return false;
        }
    }
    public void setPath(Path path, boolean holdEnd){
        autoDriveSubsystem.followPath(path, holdEnd);
    }
    public void setPathChain(PathChain pathChain, boolean holdEnd){
        autoDriveSubsystem.followPath(pathChain, holdEnd);
    }
}
