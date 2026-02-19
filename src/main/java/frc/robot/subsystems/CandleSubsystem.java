package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.hardware.CANdle;

public class CandleSubsystem { 
    CANdle candle = new CANdle(0);
    CANdleConfiguration config = new CANdleConfiguration();
//  config.stripType = LEDStripType.RGB;
//  config.brightnessScalar = 0.5;
     
}
