package org.firstinspires.ftc.teamcode.astrovos.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PIDFtunerH {
    DcMotorEx shooter;
    double p = 30.06;
    double f = 14;


    public void init(HardwareMap hwmap, Telemetry telemetry)   {
        shooter = hwmap.get(DcMotorEx.class,"shooter");
        telemetry.addLine("ïnitialised");
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void shooting(boolean a,boolean dpadup,boolean dpaddown,boolean dpadleft,boolean x,Telemetry telemetry)  {
        if (a)  {
            shooter.setVelocity(2800);
        }
        else {
            shooter.setVelocity(0);
        }

        if (dpadup)  {
             p = p + 0.01;
        }
        else if (dpaddown)  {
            p = p -0.01;
        }

        else if (dpadleft)  {
            f = f+0.01;

        }
        else if (x){
            f = f-0.01;


        }

        shooter.setVelocityPIDFCoefficients(p,0,0,f);
        telemetry.addData("p",p);
        telemetry.addData("f",f);
        telemetry.addData("velocity",shooter.getVelocity());


    }

}
