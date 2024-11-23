package org.firstinspires.ftc.teamcode.Commands;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants.PIDConstants;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Utils.PIDControl;

public class StrafeTo extends CommandBase {
    Drivetrain drive;
    PIDControl pid,pidDrive, pidYaw;
    double distance;
    public StrafeTo(double distance, Drivetrain drive){
        this.distance = distance;
        this.drive = drive;
        addRequirements(drive);
        pid = new PIDControl(distance, PIDConstants.Kp,PIDConstants.Ki, PIDConstants.Kd);
        pidDrive = new PIDControl(0,PIDConstants.Kp,0, 0);
        pidYaw = new PIDControl(0,PIDConstants.Kp,0, 0);
        drive.resetEncoders();
    }

    @Override
    public void initialize() {

    }
    @Override
    public void execute() {
        double out = pid.get(drive.getHorizontal());
        Log.d("driveTo",String.format("Out: %f",out));
        Log.d("driveTo",String.format("Vertical: %f",drive.getVertical()));
        Log.d("driveTo",String.format("Distance: %f",distance));
        double d = pidDrive.get(drive.getVertical());
        double yaw = pidYaw.get(-drive.getHeading(AngleUnit.DEGREES));

        drive.moveRobot(d,out,yaw);
    }
    @Override
    public void end(boolean interrupted) {
        drive.power(0);
    }
    @Override
    public boolean isFinished() {
        return pid.atSetpoint();
    }
}

