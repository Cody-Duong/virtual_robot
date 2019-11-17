package org.firstinspires.ftc.teamcode.team_classes;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Robot {
    public DcMotorGroup DCG;
    public ColorSensorGroup CSG;
    public BNOIMU IMU;

    //constructor
    public Robot() {
        DCG = new DcMotorGroup(new DcMotor[]{null,null,null,null});
        //CSG = new ColorSensorGroup(null);
        IMU = new BNOIMU(null);
    }

    public void initialize(HardwareMap HM, Telemetry T) {
        //T.addData("Status", "1");
        //T.update();
        DCG.initialize(HM, T);
        //T.addData("Status", "2");
        //T.update();
        //CSG.initialize(HM, T);
        //T.addData("Status", "3");
        //T.update();
        IMU.initialize(HM, T);
        //T.addData("Status", "done");
        //T.update();
    }
}
