package org.firstinspires.ftc.teamcode.astrovos.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.astrovos.util.Constants;

public class DriveSubsystem {

    private final DcMotorEx lf;
    private final DcMotorEx rf;
    private final DcMotorEx lb;
    private final DcMotorEx rb;

    public DriveSubsystem(HardwareMap hardwareMap) {

        lf = hardwareMap.get(DcMotorEx.class, Constants.Hardware.LF_NAME);
        rf = hardwareMap.get(DcMotorEx.class, Constants.Hardware.RF_NAME);
        lb = hardwareMap.get(DcMotorEx.class, Constants.Hardware.LB_NAME);
        rb = hardwareMap.get(DcMotorEx.class, Constants.Hardware.RB_NAME);

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        rf.setDirection(DcMotorSimple.Direction.FORWARD);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);
        rb.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void drive(
            double driverX,
            double driverY,
            double driverTurn,
            boolean precisionMode,
            double fieldHeadingRadians
    ) {

        if (!Constants.Drive.DRIVE_ENABLED) {
            stop();
            return;
        }

        double x =
                shape(deadband(driverX)
                        * Constants.Drive.STRAFE_MULTIPLIER);

        double y =
                shape(deadband(driverY));

        double turn =
                shape(deadband(driverTurn));

        double cos =
                Math.cos(-fieldHeadingRadians);

        double sin =
                Math.sin(-fieldHeadingRadians);

        double robotX =
                x * cos - y * sin;

        double robotY =
                x * sin + y * cos;

        double lfPower =
                robotY + robotX + turn;

        double rfPower =
                robotY - robotX - turn;

        double lbPower =
                robotY - robotX + turn;

        double rbPower =
                robotY + robotX - turn;

        double max =
                Math.max(
                        1.0,
                        Math.max(
                                Math.max(
                                        Math.abs(lfPower),
                                        Math.abs(rfPower)
                                ),
                                Math.max(
                                        Math.abs(lbPower),
                                        Math.abs(rbPower)
                                )
                        )
                );

        double scale =
                precisionMode
                        ? Constants.Drive.PRECISION_SPEED
                        : Constants.Drive.NORMAL_SPEED;

        lf.setPower(Range.clip(lfPower / max * scale, -1, 1));
        rf.setPower(Range.clip(rfPower / max * scale, -1, 1));
        lb.setPower(Range.clip(lbPower / max * scale, -1, 1));
        rb.setPower(Range.clip(rbPower / max * scale, -1, 1));
    }

    public void stop() {

        lf.setPower(0);
        rf.setPower(0);
        lb.setPower(0);
        rb.setPower(0);
    }

    private double deadband(double value) {

        return Math.abs(value) < Constants.Drive.DEADBAND
                ? 0
                : value;
    }

    private double shape(double value) {

        return Math.signum(value)
                * value
                * value;
    }
}
