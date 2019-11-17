package org.firstinspires.ftc.teamcode.team_classes;

import com.qualcomm.robotcore.hardware.DcMotor;

public class DcMotorGroup {
    //Properties
    public DcMotor[] DcMotors;
    public int DcMotorCount;

    //Constructor
    public DcMotorGroup(DcMotor[] DcMotorArray) {
        DcMotors = DcMotorArray;
        DcMotorCount = DcMotorArray.length;
    }

    //setPower to all of the motors
    public void setPower(double[] power) {
        for(int i=0; i<DcMotorCount; i++) {
            this.DcMotors[i].setPower(power[i]);
        }
    }
}
