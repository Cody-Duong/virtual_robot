package org.firstinspires.ftc.teamcode.team_classes;

import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Gyro {
    public GyroSensor Sensor;
    Gyro(GyroSensor GyroSensor){
        Sensor = GyroSensor;
    }

    public void initialize(HardwareMap Hmap, Telemetry Tm) {
        this.Sensor = Hmap.get(GyroSensor.class, "gyro_sensor");
        this.Sensor.init();
        //this.Sensor.isCalibrating();
        //while(this.Sensor.isCalibrating()){}
    }
}
