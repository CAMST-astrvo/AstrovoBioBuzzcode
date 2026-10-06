package org.firstinspires.ftc.teamcode.astrovos.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class PIDFtuner extends OpMode {
    PIDFtunerH hardware = new PIDFtunerH();
    @Override
    public void init() {
        hardware.init(hardwareMap,telemetry);


    }

    @Override
    public void loop() {
        hardware.shooting(gamepad1.a, gamepad1.dpad_up, gamepad1.dpad_down,
                gamepad1.dpad_left, gamepad1.dpad_right, telemetry);

        telemetry.update();


    }
}
