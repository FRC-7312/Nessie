package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class RevBlinkin extends SubsystemBase {

  private final Spark blinkin;
  private static RevBlinkin instance;

  private double currentPattern = 0.99; // default solid black/off
  private double flashEndTime = -1;
  private BlinkinPattern previousPattern = BlinkinPattern.BLACK;

  private double altFlashEndTime = -1;
  private double altFlashInterval = 0.1; // default toggle speed
  private double nextToggleTime = -1;
  private BlinkinPattern altColor1 = BlinkinPattern.BLACK;
  private BlinkinPattern altColor2 = BlinkinPattern.BLACK;
  private BlinkinPattern altEndColor = BlinkinPattern.BLACK;
  private boolean showingFirst = true;

  public enum BlinkinPattern {
    // Fixed Palette Patterns
    RAINBOW_RAINBOW_PALETTE(-0.99),
    RAINBOW_PARTY_PALETTE(-0.97),
    RAINBOW_OCEAN_PALETTE(-0.95),
    RAINBOW_LAVA_PALETTE(-0.93),
    RAINBOW_FOREST_PALETTE(-0.91),
    RAINBOW_WITH_GLITTER(-0.89),
    CONFETTI(-0.87),
    SHOT_RED(-0.85),
    SHOT_BLUE(-0.83),
    SHOT_WHITE(-0.81),
    SINELON_RAINBOW_PALETTE(-0.79),
    SINELON_PARTY_PALETTE(-0.77),
    SINELON_OCEAN_PALETTE(-0.75),
    SINELON_LAVA_PALETTE(-0.73),
    SINELON_FOREST_PALETTE(-0.71),
    BPM_RAINBOW_PALETTE(-0.69),
    BPM_PARTY_PALETTE(-0.67),
    BPM_OCEAN_PALETTE(-0.65),
    BPM_LAVA_PALETTE(-0.63),
    BPM_FOREST_PALETTE(-0.61),
    FIRE_MEDIUM(-0.59),
    FIRE_LARGE(-0.57),
    TWINKLES_RAINBOW(-0.55),
    TWINKLES_PARTY(-0.53),
    TWINKLES_OCEAN(-0.51),
    TWINKLES_LAVA(-0.49),
    TWINKLES_FOREST(-0.47),
    COLOR_WAVES_RAINBOW(-0.45),
    COLOR_WAVES_PARTY(-0.43),
    COLOR_WAVES_OCEAN(-0.41),
    COLOR_WAVES_LAVA(-0.39),
    COLOR_WAVES_FOREST(-0.37),
    LARSON_SCANNER_RED(-0.35),
    LARSON_SCANNER_GRAY(-0.33),
    LIGHT_CHASE_RED(-0.31),
    LIGHT_CHASE_BLUE(-0.29),
    LIGHT_CHASE_GRAY(-0.27),
    HEARTBEAT_RED(-0.25),
    HEARTBEAT_BLUE(-0.23),
    HEARTBEAT_WHITE(-0.21),
    HEARTBEAT_GRAY(-0.19),
    BREATH_RED(-0.17),
    BREATH_BLUE(-0.15),
    BREATH_GRAY(-0.13),
    STROBE_RED(-0.11),
    STROBE_BLUE(-0.09),
    STROBE_GOLD(-0.07),
    STROBE_WHITE(-0.05),

    // Color 1 Patterns
    C1_END_TO_END_BLEND_TO_BLACK(-0.03),
    C1_LARSON_SCANNER(-0.01),
    C1_LIGHT_CHASE(0.01),
    C1_HEARTBEAT_SLOW(0.03),
    C1_HEARTBEAT_MEDIUM(0.05),
    C1_HEARTBEAT_FAST(0.07),
    C1_BREATH_SLOW(0.09),
    C1_BREATH_FAST(0.11),
    C1_SHOT(0.13),
    C1_STROBE(0.15),

    // Color 2 Patterns
    C2_END_TO_END_BLEND_TO_BLACK(0.17),
    C2_LARSON_SCANNER(0.19),
    C2_LIGHT_CHASE(0.21),
    C2_HEARTBEAT_SLOW(0.23),
    C2_HEARTBEAT_MEDIUM(0.25),
    C2_HEARTBEAT_FAST(0.27),
    C2_BREATH_SLOW(0.29),
    C2_BREATH_FAST(0.31),
    C2_SHOT(0.33),
    C2_STROBE(0.35),

    // Color 1 & 2 Patterns
    SPARKLE_C1_ON_C2(0.37),
    SPARKLE_C2_ON_C1(0.39),
    COLOR_GRADIENT_C1_TO_C2(0.41),
    BPM_C1_AND_C2(0.43),
    END_TO_END_BLEND_C1_TO_C2(0.45),
    END_TO_END_BLEND(0.47),
    SOLID_C1_AND_C2(0.49),
    TWINKLES_C1_AND_C2(0.51),
    COLOR_WAVES_C1_AND_C2(0.53),
    SINELON_C1_AND_C2(0.55),

    // Solid Colors
    HOT_PINK(0.57),
    DARK_RED(0.59),
    RED(0.61),
    RED_ORANGE(0.63),
    ORANGE(0.65),
    GOLD(0.67),
    YELLOW(0.69),
    LAWN_GREEN(0.71),
    LIME(0.73),
    DARK_GREEN(0.75),
    GREEN(0.77),
    BLUE_GREEN(0.79),
    AQUA(0.81),
    SKY_BLUE(0.83),
    DARK_BLUE(0.85),
    BLUE(0.87),
    BLUE_VIOLET(0.89),
    VIOLET(0.91),
    WHITE(0.93),
    GRAY(0.95),
    DARK_GRAY(0.97),
    BLACK(0.99);

    public final double pwmValue;

    BlinkinPattern(double value) {
      this.pwmValue = value;
    }
  }

  private RevBlinkin() {
    blinkin = new Spark(Constants.ID.BLINKIN_ID);
    System.out.println("RevBlinkin subsystem initialized");
    setPattern(BlinkinPattern.RED);
  }

  /**
   * Flash between two colors for a given duration, then end on a color.
   * @param color1 First flash color
   * @param color2 Second flash color
   * @param intervalSeconds How long each color is shown before toggling
   * @param durationSeconds Total flash time
   * @param endColor The color/pattern to show when finished
   */
  public void flashAlternate(BlinkinPattern color1, BlinkinPattern color2,
                             double intervalSeconds, double durationSeconds,
                             BlinkinPattern endColor) {
    altColor1 = color1;
    altColor2 = color2;
    altEndColor = endColor;
    altFlashInterval = intervalSeconds;
    altFlashEndTime = Timer.getFPGATimestamp() + durationSeconds;
    nextToggleTime = Timer.getFPGATimestamp() + altFlashInterval;
    showingFirst = true;
    setPattern(altColor1);
  }

  public static RevBlinkin getInstance() {
    if (instance == null) {
      instance = new RevBlinkin();
    }
    return instance;
  }

  public void setPattern(BlinkinPattern pattern) {
    blinkin.set(pattern.pwmValue);
    currentPattern = pattern.pwmValue;
  }

  public void flashColor(BlinkinPattern flashPattern, double durationSeconds, BlinkinPattern endPattern) {
    setPattern(flashPattern);
    flashEndTime = Timer.getFPGATimestamp() + durationSeconds;
    previousPattern = endPattern; // store where we should end
  }

  public BlinkinPattern getPattern() {
    for (BlinkinPattern p : BlinkinPattern.values()) {
      if (p.pwmValue == currentPattern) return p;
    }
    return BlinkinPattern.BLACK;
  }

  @Override
  public void periodic() {
    double now = Timer.getFPGATimestamp();

    // One-shot flash
    if (flashEndTime > 0 && now >= flashEndTime) {
      setPattern(previousPattern);
      flashEndTime = -1;
    }

    // Alternating flash
    if (altFlashEndTime > 0) {
      if (now >= altFlashEndTime) {
        // Done flashing, set final end color
        setPattern(altEndColor);
        altFlashEndTime = -1;
      } else if (now >= nextToggleTime) {
        // Toggle between colors
        showingFirst = !showingFirst;
        setPattern(showingFirst ? altColor1 : altColor2);
        nextToggleTime = now + altFlashInterval;
      }
    }
  }

}
