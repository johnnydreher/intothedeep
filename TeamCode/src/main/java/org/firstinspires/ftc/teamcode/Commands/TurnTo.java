package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants.PIDConstants;
import org.firstinspires.ftc.teamcode.Constants.PIDConstantsTurn;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Utils.PIDControl;

public class TurnTo extends CommandBase {
    Drivetrain drive;
    PIDControl pid;
    double angle;
    ElapsedTime to = new ElapsedTime();
    public TurnTo(double angle, Drivetrain drive){
        this.angle = angle;
        this.drive = drive;
        addRequirements(drive);
        drive.resetEncoders();
        pid = new PIDControl(angle, PIDConstantsTurn.Kp,PIDConstantsTurn.Ki, PIDConstantsTurn.Kd);
    }

    @Override
    public void initialize() {
        to.reset();;
    }
    @Override
    public void execute() {
        double out = pid.get(drive.getHeading(AngleUnit.DEGREES));
        drive.driveFieldCentric(0,0,-out);
    }
    @Override
    public void end(boolean interrupted) {
        drive.power(0);
    }
    @Override
    public boolean isFinished() {
        return pid.atSetpoint() || to.milliseconds()>750;
    }
}

