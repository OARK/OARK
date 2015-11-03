/*
 * OARK Controller Software.
 *
 * Copyright (c) 2015 Open Academic Robot Kit.
 */

package org.oarkit.controller.ui;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Rect;
import android.util.AttributeSet;
import android.view.MotionEvent;
import android.view.View;

import java.util.ArrayList;

public class ControllerStick extends View implements IRobotControl {
    public enum Axes {
        X, Y, BOTH;
    }

    private int backColor = Color.BLACK;
    private int pointerColor = Color.RED;

    private boolean isTouching = false;
    private float touchX, touchY;
    private float circleRad = 150f;

    private Paint circlePainter;
    private Paint mTextPainter;
    private Rect mTextBounds;

    private boolean effects = true;
    private Axes axes = Axes.BOTH;
    private boolean transparent = true;

    private String mTitle;

    private ArrayList<AnalogStickListener> listeners =
            new ArrayList<>();

    /*
      Implement this interface if you wish to receive notifications on
      the analog stick moving.
     */
    public interface AnalogStickListener {
        void onAnalogStickChange(float x, float y);
    }

    public void addAnalogStickListener(AnalogStickListener asl) {
        listeners.add(asl);
    }

    public ControllerStick(Context context) {
        super(context);
        init(null, 0);
        mTitle = "";
        mTextPainter = new Paint();

        mTextPainter.setStyle(Paint.Style.FILL);
        mTextPainter.setColor(Color.WHITE);
        mTextPainter.setTextSize(20);

        mTextBounds = new Rect();
    }

    public ControllerStick(Context context, AttributeSet attrs) {
        super(context, attrs);
        init(attrs, 0);
    }

    public ControllerStick(Context context, AttributeSet attrs, int defStyle) {
        super(context, attrs, defStyle);
        init(attrs, defStyle);
    }

    private void init(AttributeSet attrs, int defStyle) {
        /* Create default painting object */
        circlePainter = new Paint();
        circlePainter.setColor(pointerColor);

        touchX = getWidth() / 2;
        touchY = getHeight() / 2;
    }

    @Override
    protected void onDraw(Canvas canvas) {
        super.onDraw(canvas);

        int width = getWidth();
        int height = getHeight();

        /* This needs to be here for when drawing occurs before action */
        if(!isTouching) {
            touchX = width / 2;
            touchY = height / 2;
        }

        if(!transparent) {
            canvas.drawColor(backColor);
        }

        if(effects) {
            /* Save current transformation matrix */
            canvas.save();
            double angle;
            double tilt;

            angle = Math.toDegrees(Math.atan2(getAnalogY(), getAnalogX()));
            tilt = 1 - (Math.hypot(touchX - width / 2, touchY - height / 2) /
                        Math.hypot(width / 2, height / 2));

            canvas.translate(touchX, touchY);
            canvas.rotate((float) -angle);
            canvas.scale((float) tilt, 1);

            canvas.drawCircle(0, 0, circleRad, circlePainter);

            /* Restore old transformation matrix */
            canvas.restore();
        }
        else {
            canvas.drawCircle(touchX, touchY, circleRad, circlePainter);
        }

        // If there's a label, draw it.
        mTextPainter.getTextBounds(mTitle, 0, mTitle.length(), mTextBounds);

        int middleWidth = width / 2;
        int middleHeight = height / 2;

        canvas.drawText(mTitle, middleWidth - mTextBounds.width(), middleHeight, mTextPainter);
    }


    @Override
    public boolean onTouchEvent(MotionEvent me) {
        int width = getWidth();
        int height = getHeight();

        /* Get only required axes */
        if(axes != Axes.Y) {
            touchX = me.getX();
        }
        if(axes != Axes.X) {
            touchY = me.getY();
        }

        /* Check for down/up */
        if(me.getAction() == MotionEvent.ACTION_DOWN) {
            isTouching = true;
        }
        else if(me.getAction() == MotionEvent.ACTION_UP) {
            isTouching = false;
            touchX = width / 2;
            touchY = height / 2;
        }

        /* Ensure our cursor doesn't escape view bounds */
        if(touchX - circleRad < 0) touchX = circleRad;
        if(touchX + circleRad > width) touchX = width - circleRad;
        if(touchY - circleRad < 0) touchY = circleRad;
        if(touchY + circleRad > height) touchY = height - circleRad;

        for(AnalogStickListener asl : listeners) {
            asl.onAnalogStickChange((float)getAnalogX(), (float)getAnalogY());
        }

        invalidate();

        return true;
    }

    public void setAxes(Axes inAxes) {
        axes = inAxes;
    }

    public void setTitle(String inTitle) {
        mTitle = inTitle;
    }

    /* We have to use an effective width/height because the range of the pointer is
     * restricted so that it doesn't go out of bands.
     */
    public double getAnalogX()
    {
        double effectiveWidth = getWidth() - 2 * circleRad;
        return (touchX / (getWidth() / 2.0) - 1.0) * (getWidth() / effectiveWidth);
    }

    public double getAnalogY()
    {
        double effectiveHeight = getHeight() - 2 * circleRad;

        return -(touchY / (getHeight() / 2.0) - 1.0) * (getHeight() / effectiveHeight);
    }

    public float[] getValues() {
        return new float[]{(float)getAnalogX(), (float)getAnalogY()};
    }
}
