package sep_17.fourwwcontrol;

import sep_17.fourwwcontrol.util.SystemUiHider;

import android.app.Activity;
import android.os.Bundle;
import android.util.Log;
import android.widget.SeekBar;

import java.util.Timer;
import java.util.TimerTask;


public class ControlActivity extends Activity {
    private Timer netTimer;


    private String targetIP = "192.168.0.1";

    private Talker talker;

    private final float speedEpsilon = 0.1f;
    private final float posEpsilon = 0.05f;

    private double curHand = 0;
    private double curWrist = 0;
    private double curElbow = 0;

    private double curLeft = 0;
    private double curRight = 0;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        setContentView(R.layout.activity_control);

        final AnalogStickView leftAnalog = (AnalogStickView) findViewById(R.id.left_analog_stick);
        final AnalogStickView rightAnalog = (AnalogStickView) findViewById(R.id.right_analog_stick);
        final SeekBar handSeek = (SeekBar) findViewById(R.id.handSeek);
        final SeekBar wristSeek = (SeekBar) findViewById(R.id.wristSeek);
        final SeekBar elbowSeek = (SeekBar) findViewById(R.id.elbowSeek);

        targetIP = getIntent().getStringExtra("tim.is.a.legend.IP_ADDRESS");

        try {
            talker = new Talker(targetIP);
        }
        catch(Exception e) {
            Log.e("FOURWW", "Exception occured: " + e.getMessage());
            throw new RuntimeException(e);
        }

        netTimer = new Timer();
        netTimer.schedule(new TimerTask() {
            public void run() {
                FourWWMsg fwm = new FourWWMsg(FourWWMsg.NOOP,
                                              (byte)0,
                                              (byte)0,
                                              1000);
                try {
                    final float elbowPos = (float)elbowSeek.getProgress() / elbowSeek.getMax();
                    final float wristPos = (float)wristSeek.getProgress() / wristSeek.getMax();
                    final float handPos = (float)handSeek.getProgress() / handSeek.getMax();

                    if (Math.abs(leftAnalog.getAnalogY() - curLeft) > speedEpsilon) {
                        fwm.setType(fwm.LEFT_GO);
                        fwm.setVal((byte)(leftAnalog.getAnalogY() * 10));
                        talker.send(fwm);

                        curLeft = leftAnalog.getAnalogY();

                    }
                    if (Math.abs(rightAnalog.getAnalogY() - curRight) > speedEpsilon) {
                        fwm.setType(fwm.RIGHT_GO);
                        fwm.setVal((byte)(rightAnalog.getAnalogY() * 10));
                        talker.send(fwm);

                        curRight = rightAnalog.getAnalogY();
                    }
                    if (Math.abs(elbowPos - curElbow) > posEpsilon){
                        fwm.setType(fwm.ARM_GO);
                        fwm.setVal((byte)(elbowPos * 127));
                        talker.send(fwm);

                        curElbow = elbowPos;
                    }
                    if (Math.abs(wristPos - curWrist) > posEpsilon){
                        fwm.setType(fwm.WRIST_GO);
                        fwm.setVal((byte)(wristPos * 127));
                        talker.send(fwm);

                        curWrist = wristPos;
                    }
                    if (Math.abs(handPos - curHand) > posEpsilon){
                        fwm.setType(fwm.HAND_GO);
                        fwm.setVal((byte)(handPos * 127));
                        talker.send(fwm);

                        curHand = handPos;
                    }
                }
                catch(Exception e) {
                    Log.e("FOURWW", "Couldn't send over network: " + e.getMessage());
                    throw new RuntimeException(e);
                }
            }
        }, 3000, 50);
    }
}
