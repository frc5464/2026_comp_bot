package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;

public class CandleSubsystem { 
    CANdle candle;
    CANdleConfiguration config = new CANdleConfiguration();
    RainbowAnimation rainbow = new RainbowAnimation(0, 200);

    StrobeAnimation strobe = new StrobeAnimation(0, 200);

    public CandleSubsystem(){
        strobe.Color = RGBWColor.fromHSV(226, 85, 40);
        candle = new CANdle(52);
        strobe.FrameRate = 15;
        candle.setControl(strobe);
    }
//  config.brightnessScalar = 0.5;
     
}
