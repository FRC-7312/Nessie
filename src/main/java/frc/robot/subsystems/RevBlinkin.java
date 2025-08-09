package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class RevBlinkin extends SubsystemBase {
  Spark blinkin;

  /** Creates a new RevBlinkin. */
  public RevBlinkin() {
    blinkin = new Spark(Constants.ID.BLINKIN_ID);
    System.out.println("RevBlinkin subsystem initialized");
  }

  public void rainbowPalettePattern() { //Rainbow, Rainbow Palette Pattern Density Speed Brightness
    blinkin.set(-0.99);
    }

    public void partyPalettePattern() { //Rainbow, Party Palette Pattern Density Speed Brightness
    blinkin.set(-0.97);
    }

    public void oceanPalettePattern() { //Rainbow, Ocean Palette Pattern Density Speed Brightness
    blinkin.set(-0.95);
    }

    public void lavPalettePattern() { //Rainbow, Lav Palette Pattern Density Speed Brightness
    blinkin.set(-0.93);
    }

    public void forestPalettePattern() { //Rainbow, Forest Palette Pattern Density Speed Brightness
    blinkin.set(-0.91);
    }

    public void rainbowWithGlitterPattern() { //Rainbow with Glitter Pattern Density Speed Brightness
    blinkin.set(-0.89);
    }

    public void confettiPattern() { //Confetti Pattern Density Speed Brightness
    blinkin.set(-0.87);
    }

    public void shotRedPattern() { //Shot, Red - - Brightness
    blinkin.set(-0.85);
    }

    public void shotBluePattern() { //Shot, Blue - - Brightness
    blinkin.set(-0.83);
    }

    public void shotWhitePattern() { //Shot, White - - Brightness
    blinkin.set(-0.81);
    }

    public void sinelonRainbowPalettePattern() { //Sinelon, Rainbow Palette Pattern Density Speed Brightness
    blinkin.set(-0.79);
    }

    public void sinelonPartyPalettePattern() { //Sinelon, Party Palette Pattern Density Speed Brightness
    blinkin.set(-0.77);
    }

    public void sinelonOceanPalettePattern() { //Sinelon, Ocean Palette Pattern Density Speed Brightness
    blinkin.set(-0.75);
    }

    public void sinelonLavaPalettePattern() { //Sinelon, Lava Palette Pattern Density Speed Brightness
    blinkin.set(-0.73);
    }

    public void sinelonForestPalettePattern() { //Sinelon, Forest Palette Pattern Density Speed Brightness
    blinkin.set(-0.71);
    }

    public void beatsPerMinuteRainbowPalettePattern() { //Beats per Minute, Rainbow Palette Pattern Density Speed Brightness
    blinkin.set(-0.69);
    }

    public void beatsPerMinutePartyPalettePattern() { //Beats per Minute, Party Palette Pattern Density Speed Brightness
    blinkin.set(-0.67);
    }

    public void beatsPerMinuteOceanPalettePattern() { //Beats per Minute, Ocean Palette Pattern Density Speed Brightness
    blinkin.set(-0.65);
    }

    public void beatsPerMinuteLavaPalettePattern() { //Beats per Minute, Lava Palette Pattern Density Speed Brightness
    blinkin.set(-0.63);
    }

    public void beatsPerMinuteForestPalettePattern() { //Beats per Minute, Forest Palette Pattern Density Speed Brightness
    blinkin.set(-0.61);
    }

    public void fireMediumPattern() { //Fire, Medium - - Brightness
    blinkin.set(-0.59);
    }

    public void fireLargePattern() { //Fire, Large - - Brightness
    blinkin.set(-0.57);
    }

    public void twinklesRainbowPalettePattern() { //Twinkles, Rainbow Palette - - Brightness
    blinkin.set(-0.55);
    }

    public void twinklesPartyPalettePattern() { //Twinkles, Party Palette - - Brightness
    blinkin.set(-0.53);
    }

    public void twinklesOceanPalettePattern() { //Twinkles, Ocean Palette - - Brightness
    blinkin.set(-0.51);
    }

    public void twinklesLavaPalettePattern() { //Twinkles, Lava Palette - - Brightness
    blinkin.set(-0.49);
    }

    public void twinklesForestPalettePattern() { //Twinkles, Forest Palette - - Brightness
    blinkin.set(-0.47);
    }

    public void colorWavesRainbowPalettePattern() { //Color Waves, Rainbow Palette - - Brightness
    blinkin.set(-0.45);
    }

    public void colorWavesPartyPalettePattern() { //Color Waves, Party Palette - - Brightness
    blinkin.set(-0.43);
    }

    public void colorWavesOceanPalettePattern() { //Color Waves, Ocean Palette - - Brightness
    blinkin.set(-0.41);
    }

    public void colorWavesLavaPalettePattern() { //Color Waves, Lava Palette - - Brightness
    blinkin.set(-0.39);
    }

    public void colorWavesForestPalettePattern() { //Color Waves, Forest Palette - - Brightness
    blinkin.set(-0.37);
    }

    public void larsonScannerRedPattern() { //Larson Scanner, Red Pattern Width Speed Brightness
    blinkin.set(-0.35);
    }

    public void larsonScannerGrayPattern() { //Larson Scanner, Gray Pattern Width Speed Brightness
    blinkin.set(-0.33);
    }

    public void lightChaseRedPattern() { //Light Chase, Red Dimming Speed Brightness
    blinkin.set(-0.31);
    }

    public void lightChaseBluePattern() { //Light Chase, Blue Dimming Speed Brightness
    blinkin.set(-0.29);
    }

    public void lightChaseGrayPattern() { //Light Chase, Gray Dimming Speed Brightness
    blinkin.set(-0.27);
    }

    public void heartbeatRedPattern() { //Heartbeat, Red - - Brightness
    blinkin.set(-0.25);
    }

    public void heartbeatBluePattern() { //Heartbeat, Blue - - Brightness
    blinkin.set(-0.23);
    }

    public void heartbeatWhitePattern() { //Heartbeat, White - - Brightness
    blinkin.set(-0.21);
    }

    public void heartbeatGrayPattern() { //Heartbeat, Gray - - Brightness
    blinkin.set(-0.19);
    }

    public void breathRedPattern() { //Breath, Red - - Brightness
    blinkin.set(-0.17);
    }

    public void breathBluePattern() { //Breath, Blue - - Brightness
    blinkin.set(-0.15);
    }

    public void breathGrayPattern() { //Breath, Gray - - Brightness
    blinkin.set(-0.13);
    }

    public void strobeRedPattern() { //Strobe, Red - - Brightness
    blinkin.set(-0.11);
    }

    public void strobeBluePattern() { //Strobe, Blue - - Brightness
    blinkin.set(-0.09);
    }

    public void strobeGoldPattern() { //Strobe, Gold - - Brightness
    blinkin.set(-0.07);
    }

    public void strobeWhitePattern() { //Strobe, White - - Brightness
    blinkin.set(-0.05);
    }

    public void endToEndBlendToBlackColor1Pattern() { //End to End Blend to Black - - Brightness Color 1
    blinkin.set(-0.03);
    }

    public void larsonScannerColor1Pattern() { //Larson Scanner Pattern Width Speed Brightness Color 1
    blinkin.set(-0.01);
    }

    public void lightChaseColor1Pattern() { //Light Chase Dimming Speed Brightness Color 1
    blinkin.set(0.01);
    }

    public void heartbeatSlowColor1Pattern() { //Heartbeat Slow - - Brightness Color 1
    blinkin.set(0.03);
    }

    public void heartbeatMediumColor1Pattern() { //Heartbeat Medium - - Brightness Color 1
    blinkin.set(0.05);
    }

    public void heartbeatFastColor1Pattern() { //Heartbeat Fast - - Brightness Color 1
    blinkin.set(0.07);
    }

    public void breathSlowColor1Pattern() { //Breath Slow - - Brightness Color 1
    blinkin.set(0.09);
    }

    public void breathFastColor1Pattern() { //Breath Fast - - Brightness Color 1
    blinkin.set(0.11);
    }

    public void shotColor1Pattern() { //Shot - - Brightness Color 1
    blinkin.set(0.13); //Corrected value from 0.13 to 0.15 based on table and next value.
    }

    public void strobeColor1Pattern() { //Strobe - - Brightness Color 1
    blinkin.set(0.15); //Corrected value from 0.15 to 0.17 based on table and next value.
    }

    public void endToEndBlendToBlackColor2Pattern() { //End to End Blend to Black - - Brightness Color 2
    blinkin.set(0.17);
    }

    public void larsonScannerColor2Pattern() { //Larson Scanner Pattern Width Speed Brightness Color 2
    blinkin.set(0.19);
    }

    public void lightChaseColor2Pattern() { //Light Chase Dimming Speed Brightness Color 2
    blinkin.set(0.21);
    }

    public void heartbeatSlowColor2Pattern() { //Heartbeat Slow - - Brightness Color 2
    blinkin.set(0.23);
    }

    public void heartbeatMediumColor2Pattern() { //Heartbeat Medium - - Brightness Color 2
    blinkin.set(0.25);
    }

    public void heartbeatFastColor2Pattern() { //Heartbeat Fast - - Brightness Color 2
    blinkin.set(0.27);
    }

    public void breathSlowColor2Pattern() { //Breath Slow - - Brightness Color 2
    blinkin.set(0.29);
    }

    public void breathFastColor2Pattern() { //Breath Fast - - Brightness Color 2
    blinkin.set(0.31);
    }

    public void shotColor2Pattern() { //Shot - - Brightness Color 2
    blinkin.set(0.33);
    }

    public void strobeColor2Pattern() { //Strobe - - Brightness Color 2
    blinkin.set(0.35);
    }

    public void sparkleColor1OnColor2Pattern() { //Sparkle, Color 1 on Color 2 - - Brightness Color 1 and 2
    blinkin.set(0.37);
    }

    public void sparkleColor2OnColor1Pattern() { //Sparkle, Color 2 on Color 1 - - Brightness Color 1 and 2
    blinkin.set(0.39);
    }

    public void colorGradientColor1And2Pattern() { //Color Gradient, Color 1 and 2 - - Brightness Color 1 and 2
    blinkin.set(0.41);
    }

    public void beatsPerMinuteColor1And2Pattern() { //Beats per Minute, Color 1 and 2 Pattern Density Speed Brightness Color 1 and 2
    blinkin.set(0.43);
    }

    public void endToEndBlendColor1To2Pattern() { //End to End Blend, Color 1 to 2 - - Brightness Color 1 and 2
    blinkin.set(0.45);
    }

    public void endToEndBlendColor1And2Pattern() { //End to End Blend - - Brightness Color 1 and 2
    blinkin.set(0.47);
    }

    public void color1AndColor2NoBlendingPattern() { //Color 1 and Color 2 no blending (Setup Pattern) - - Brightness Color 1 and 2
    blinkin.set(0.49);
    }

    public void twinklesColor1And2Pattern() { //Twinkles, Color 1 and 2 - - Brightness Color 1 and 2
    blinkin.set(0.51);
    }

    public void colorWavesColor1And2Pattern() { //Color Waves, Color 1 and 2 - - Brightness Color 1 and 2
    blinkin.set(0.53);
    }

    public void sinelonColor1And2Pattern() { //Sinelon, Color 1 and 2 Pattern Density Speed Brightness Color 1 and 2
    blinkin.set(0.55);
    }

    public void hotPinkSolidColor() { //Solid Colors Hot Pink - - Brightness
    blinkin.set(0.57);
    }

    public void darkRedSolidColor() { //Solid Colors Dark red - - Brightness
    blinkin.set(0.59);
    }

    public void redSolidColor() { //Solid Colors Red - - Brightness
    blinkin.set(0.61);
    }

    public void redOrangeSolidColor() { //Solid Colors Red Orange - - Brightness
    blinkin.set(0.63);
    }

    public void orangeSolidColor() { //Solid Colors Orange - - Brightness
    blinkin.set(0.65);
    }

    public void goldSolidColor() { //Solid Colors Gold - - Brightness
    blinkin.set(0.67);
    }

    public void yellowSolidColor() { //Solid Colors Yellow - - Brightness
    blinkin.set(0.69);
    }

    public void lawnGreenSolidColor() { //Solid Colors Lawn Green - - Brightness
    blinkin.set(0.71);
    }

    public void limeSolidColor() { //Solid Colors Lime - - Brightness
    blinkin.set(0.73);
    }

    public void darkGreenSolidColor() { //Solid Colors Dark Green - - Brightness
    blinkin.set(0.75);
    }

    public void greenSolidColor() { //Solid Colors Green - - Brightness
    blinkin.set(0.77);
    }

    public void blueGreenSolidColor() { //Solid Colors Blue Green - - Brightness
    blinkin.set(0.79);
    }

    public void aquaSolidColor() { //Solid Colors Aqua - - Brightness
    blinkin.set(0.81);
    }

    public void skyBlueSolidColor() { //Solid Colors Sky Blue - - Brightness
    blinkin.set(0.83);
    }

    public void darkBlueSolidColor() { //Solid Colors Dark Blue - - Brightness
    blinkin.set(0.85);
    }

    public void blueSolidColor() { //Solid Colors Blue - - Brightness
    blinkin.set(0.87);
    }

    public void blueVioletSolidColor() { //Solid Colors Blue Violet - - Brightness
    blinkin.set(0.89);
    }

    public void violetSolidColor() { //Solid Colors Violet - - Brightness
    blinkin.set(0.91);
    }

    public void whiteSolidColor() { //Solid Colors White - - Brightness
    blinkin.set(0.93);
    }

    public void graySolidColor() { //Solid Colors Gray - - Brightness
    blinkin.set(0.95);
    }

    public void darkGraySolidColor() { //Solid Colors Dark Gray - - Brightness
    blinkin.set(0.97);
    }

    public void blackSolidColor() { //Solid Colors Black - - Brightness
    blinkin.set(0.99);
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
