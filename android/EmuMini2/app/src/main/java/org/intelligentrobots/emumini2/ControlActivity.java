package org.intelligentrobots.emumini2;

import android.app.Activity;
import android.os.Bundle;
import android.util.Log;
import android.view.SurfaceView;
import android.view.WindowManager;
import android.widget.SeekBar;
import android.widget.Toast;

import org.emumini2.R;


public class ControlActivity extends Activity {
    private String targetIP = "192.168.12.1";

    private Talker talker;

    private final float speedEpsilon = 0.1f;
    private final float posEpsilon = 0.05f;

    private double curHand = 0;
    private double curWrist = 0;
    private double curElbow = 0;

    private double curLeft = 0;
    private double curRight = 0;

    private SurfaceView testSurfaceView;

    private VideoRenderer testVideoRenderer;


    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        setContentView(R.layout.activity_control);

        /* Stop screen dimming */
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

        /* Initialise video */
        testSurfaceView = (SurfaceView) findViewById(R.id.robotCameraView);

        testVideoRenderer = new VideoRenderer(testSurfaceView, 5000);

        testSurfaceView.getHolder().addCallback(testVideoRenderer);

        /* Initialise controls */
        final AnalogStickView leftAnalog = (AnalogStickView) findViewById(R.id.left_analog_stick);
        final AnalogStickView rightAnalog = (AnalogStickView) findViewById(R.id.right_analog_stick);
        final SeekBar handSeek = (SeekBar) findViewById(R.id.handSeek);
        final SeekBar wristSeek = (SeekBar) findViewById(R.id.wristSeek);
        final SeekBar elbowSeek = (SeekBar) findViewById(R.id.elbowSeek);

        /* Connect to raspberry pi server */
        // targetIP = getIntent().getStringExtra("IP_ADDRESS");
        try {
            talker = new Talker(targetIP);
        }
        catch(Exception e) {
            /* TODO: Put this code in a listener and pass it to server thread */
            Log.e("EMUMINI2", "Could not connect: " + e.getMessage());
            Toast.makeText(getApplicationContext(),
                           "Could not connect to " + targetIP,
                           Toast.LENGTH_SHORT).show();
            finish();
        }

        /* Listen to changes on the left analog stick */
        leftAnalog.addAnalogStickListener(
                new AnalogStickView.AnalogStickListener() {
                    public void onAnalogStickChange(float x, float y) {
                        FourWWMsg fwm = new FourWWMsg();

                        if (Math.abs(leftAnalog.getAnalogY() - curLeft) > speedEpsilon) {
                            fwm.setType(fwm.LEFT_GO);
                            fwm.setVal((byte)(leftAnalog.getAnalogY() * 10));
                            try {
                                talker.send(fwm);
                            }
                            catch(Exception e) {
                                Log.w("EMUMINI2", "Unable to send message");
                            }

                            curLeft = leftAnalog.getAnalogY();

                        }
                        Log.e("EMUMINI2", "Left -- X: " + x + " Y: " + y);
                    }
                }
        );

        /* Listen to changes on the right analog stick */
        rightAnalog.addAnalogStickListener(
                new AnalogStickView.AnalogStickListener() {
                    public void onAnalogStickChange(float x, float y) {
                        FourWWMsg fwm = new FourWWMsg();

                        if (Math.abs(rightAnalog.getAnalogY() - curRight) > speedEpsilon) {
                            fwm.setType(fwm.RIGHT_GO);
                            fwm.setVal((byte)(rightAnalog.getAnalogY() * 10));
                            try {
                                talker.send(fwm);
                            }
                            catch(Exception e) {
                                Log.w("EMUMINI2", "Unable to send message");
                            }

                            curRight = rightAnalog.getAnalogY();
                        }

                        Log.e("EMUMINI2", "Right -- X: " + x + " Y: " + y);
                    }
                }
        );

        /* Listen to changes on the elbow seekbar */
        elbowSeek.setOnSeekBarChangeListener(
                new SeekBar.OnSeekBarChangeListener() {
                    public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                        FourWWMsg fwm = new FourWWMsg();
                        final float elbowPos = (float)progress / elbowSeek.getMax();

                        if (Math.abs(elbowPos - curElbow) > posEpsilon){
                            fwm.setType(fwm.ARM_GO);
                            fwm.setVal((byte)(elbowPos * 127));
                            try {
                                talker.send(fwm);
                            }
                            catch(Exception e) {
                                Log.w("EMUMINI2", "Unable to send message");
                            }

                            Log.e("EMUMINI2", "Elbow: " + elbowPos);
                            curElbow = elbowPos;
                        }
                    }
                    public void onStartTrackingTouch(SeekBar seekBar) {}
                    public void onStopTrackingTouch(SeekBar seekBar) {}
                }
        );
        /* Listen to changes on the wrist seekbar */
        wristSeek.setOnSeekBarChangeListener(
                new SeekBar.OnSeekBarChangeListener() {
                    public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                        FourWWMsg fwm = new FourWWMsg();
                        final float wristPos = (float)progress / wristSeek.getMax();

                        if (Math.abs(wristPos - curWrist) > posEpsilon){
                            fwm.setType(fwm.WRIST_GO);
                            fwm.setVal((byte)(wristPos * 127));
                            try {
                                talker.send(fwm);
                            }
                            catch(Exception e) {
                                Log.w("EMUMINI2", "Unable to send message");
                            }

                            Log.e("EMUMINI2", "Wrist: " + wristPos);
                            curWrist = wristPos;
                        }
                    }
                    public void onStartTrackingTouch(SeekBar seekBar) {}
                    public void onStopTrackingTouch(SeekBar seekBar) {}
                }
        );
        /* Listen to changes on the hand seekbar */
        handSeek.setOnSeekBarChangeListener(
                new SeekBar.OnSeekBarChangeListener() {
                    public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                        FourWWMsg fwm = new FourWWMsg();
                        final float handPos = (float)progress / handSeek.getMax();

                        if (Math.abs(handPos - curHand) > posEpsilon){
                            fwm.setType(fwm.HAND_GO);
                            fwm.setVal((byte)(handPos * 127));
                            try {
                                talker.send(fwm);
                            }
                            catch(Exception e) {
                                Log.w("EMUMINI2", "Unable to send message");
                            }

                            Log.e("EMUMINI2", "Hand: " + handPos);
                            curElbow = handPos;
                        }
                    }
                    public void onStartTrackingTouch(SeekBar seekBar) {}
                    public void onStopTrackingTouch(SeekBar seekBar) {}
                }
        );
    }
}
