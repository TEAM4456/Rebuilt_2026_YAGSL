package frc.robot.subsystems;
import static edu.wpi.first.units.Units.*;

import org.w3c.dom.css.RGBColor;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

    

public class LEDs extends SubsystemBase {

    private CANdle ledCandle;
    private CANdleConfiguration cfg;

    //private static final RGBWColor kGreen = new RGBWColor(0, 217, 0, 0);
    private static final RGBWColor kWhite = new RGBWColor(Color.kWhite).scaleBrightness(0.5);
    private static final RGBWColor kRed = new RGBWColor(Color.kRed).scaleBrightness(0.5);
    private static final RGBWColor kBlue = new RGBWColor(Color.kBlue).scaleBrightness(0.5);
    private static final RGBWColor kGreen = new RGBWColor(Color.kGreen).scaleBrightness(0.5);
    private static final RGBWColor kYellow = new RGBWColor(Color.kYellow).scaleBrightness(0.5);
    private static final RGBWColor kPurple = new RGBWColor(Color.kPurple).scaleBrightness(0.5);
    private static final RGBWColor kCyan = new RGBWColor(Color.kCyan).scaleBrightness(0.5);
    private static final RGBWColor kOrange = new RGBWColor(Color.kOrange).scaleBrightness(0.5);
    private static final RGBWColor kOff = new RGBWColor(0, 0, 0, 0);

    /*
     * Start and end index for LED animations.
     * 0-7 are onboard, 8-399 are an external strip.
     * CANdle supports 8 animation slots (0-7).
     */
    private static final int kSlot0StartIdx = 8;
    private static final int kSlot0EndIdx = 200;

    private static final int kSlot1StartIdx = 200;
    private static final int kSlot1EndIdx = 300;

     private enum AnimationType {
        None,
        ColorFlow,
        Fire,
        Larson,
        Rainbow,
        RgbFade,
        SingleFade,
        Strobe,
        Twinkle,
        TwinkleOff,
    }

    private AnimationType m_anim0State = AnimationType.None;
    private AnimationType m_anim1State = AnimationType.None;

    private final SendableChooser<AnimationType> m_anim0Chooser = new SendableChooser<AnimationType>();
    private final SendableChooser<AnimationType> m_anim1Chooser = new SendableChooser<AnimationType>();

    public LEDs() {
    ledCandle = new CANdle(14, CANBus.roboRIO());

     /* Configure CANdle */
        cfg = new CANdleConfiguration();
        /* set the LED strip type and brightness */
        cfg.LED.StripType = StripTypeValue.GRB;
        cfg.LED.BrightnessScalar = 0.5;
        /* disable status LED when being controlled */
        cfg.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Enabled;

        ledCandle.getConfigurator().apply(cfg);

          for (int i = 0; i < 8; ++i) {
            ledCandle.setControl(new EmptyAnimation(i));
        }
        /* set the onboard LEDs to a solid color */
        ledCandle.setControl(new SolidColor(0, 0).withColor(kWhite));
        ledCandle.setControl(new SolidColor(1, 1).withColor(kRed));
        ledCandle.setControl(new SolidColor(2, 2).withColor(kGreen));
        ledCandle.setControl(new SolidColor(3, 3).withColor(kBlue));
        ledCandle.setControl(new SolidColor(4, 4).withColor(kYellow));
        ledCandle.setControl(new SolidColor(5, 5).withColor(kPurple));
        ledCandle.setControl(new SolidColor(6, 6).withColor(kCyan));
        ledCandle.setControl(new SolidColor(7, 7).withColor(kOrange));
        ledCandle.setControl(new SolidColor(kSlot0StartIdx, kSlot0EndIdx).withColor(kRed));

        /* add animations to chooser for slot 0 */
        m_anim0Chooser.setDefaultOption("Color Flow", AnimationType.ColorFlow);
        m_anim0Chooser.addOption("Rainbow", AnimationType.Rainbow);
        m_anim0Chooser.addOption("Twinkle", AnimationType.Twinkle);
        m_anim0Chooser.addOption("Twinkle Off", AnimationType.TwinkleOff);
        m_anim0Chooser.addOption("Fire", AnimationType.Fire);

        /* add animations to chooser for slot 1 */
        m_anim1Chooser.setDefaultOption("Larson", AnimationType.Larson);
        m_anim1Chooser.addOption("RGB Fade", AnimationType.RgbFade);
        m_anim1Chooser.addOption("Single Fade", AnimationType.SingleFade);
        m_anim1Chooser.addOption("Strobe", AnimationType.Strobe);
        m_anim1Chooser.addOption("Fire", AnimationType.Fire);


    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */

     
      }

public Command setLEDsRed() {
        return run(
            () -> {
                ledCandle.setControl(new SolidColor(kSlot0StartIdx, kSlot0EndIdx).withColor(kRed));
            }
        );
     }

public Command setLEDsWhite() {
        return run(
            () -> {
                ledCandle.setControl(new SolidColor(kSlot0StartIdx, kSlot0EndIdx).withColor(kWhite));
            }
        );
     }

public Command setLEDsBlue() {
        return run(
            () -> {
                ledCandle.setControl(new SolidColor(kSlot0StartIdx, kSlot0EndIdx).withColor(kBlue));
            }
        );
     }

public Command setLEDsGreen() {
        return run(
            () -> {
                ledCandle.setControl(new SolidColor(kSlot0StartIdx, kSlot0EndIdx).withColor(kGreen));
            }
        );
     }

public Command setLEDsYellow() {
        return run(
            () -> {
                ledCandle.setControl(new SolidColor(kSlot0StartIdx, kSlot0EndIdx).withColor(kYellow));
            }
        );
     }

    @Override
     public void periodic() {

  
  }
   }