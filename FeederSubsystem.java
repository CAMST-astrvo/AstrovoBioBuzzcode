package org.firstinspires.ftc.teamcode.astrovos.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.teamcode.astrovos.util.Constants;

public class FeederSubsystem {

    private final DcMotorEx intake;

    private String intakeMode = "off";

    public FeederSubsystem(HardwareMap hardwareMap) {

        intake =
                hardwareMap.get(
                        DcMotorEx.class,
                        Constants.Hardware.INTAKE_NAME
                );



        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void update(
            boolean feedActive,
            boolean manualIntake,
            boolean reverseIntake
    ) {

        if (feedActive) {
            intake.setPower(Constants.Feeder.FEED_INTAKE_POWER);
            intakeMode = "auto feed";
        } else {

            if (reverseIntake) {
                intake.setPower(Constants.Feeder.REVERSE_INTAKE_POWER);
                intakeMode = "reverse";
            } else if (manualIntake) {
                intake.setPower(Constants.Feeder.MANUAL_INTAKE_POWER);
                intakeMode = "manual";
            } else {
                intake.setPower(0);
                intakeMode = "off";
            }
        }


    }


    public String getIntakeMode() {

        return intakeMode;
    }

}
