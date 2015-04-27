package sep_17.fourwwcontrol;

import android.content.Context;
import android.content.res.TypedArray;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.util.AttributeSet;
import android.view.View;
import android.view.MotionEvent;


/**
 * This view is of a basic analog stick. The analog stick can be
 * queried to get a value between 1.0 and -1.0 in both the X and Y
 * axes. The positive direction will be the direction of positive
 * on the x/y axis in a cartesian coordinate system rather than a
 * screen coordinate system.
 *
 * The view ignores padding.
 * The view has three attributes:
 *      backColor - The color of the analog stick background
 *      pointerColor - The color of the pointer
 *      pointerRad - The radius of the pointer circle.
 */
public class AnalogStickView extends View {
    private int backColor = Color.BLACK;
    private int pointerColor = Color.RED;

    private boolean isTouching = false;
    private float touchX, touchY;
    private float circleRad = 5.0f;
    private Paint circlePainter;

    public AnalogStickView(Context context) {
        super(context);
        init(null, 0);
    }

    public AnalogStickView(Context context, AttributeSet attrs) {
        super(context, attrs);
        init(attrs, 0);
    }

    public AnalogStickView(Context context, AttributeSet attrs, int defStyle) {
        super(context, attrs, defStyle);
        init(attrs, defStyle);
    }

    private void init(AttributeSet attrs, int defStyle) {
        // Load attributes
        final TypedArray a = getContext().obtainStyledAttributes(
                attrs, R.styleable.AnalogStickView, defStyle, 0);

        /* Retrieve styling attributes if available */
        backColor = a.getColor(R.styleable.AnalogStickView_backColor, backColor);
        pointerColor = a.getColor(R.styleable.AnalogStickView_pointerColor, pointerColor);
        circleRad = a.getFloat(R.styleable.AnalogStickView_pointerRad, circleRad);

        a.recycle();

        /* Create default painting object */
        circlePainter = new Paint();
        circlePainter.setColor(pointerColor);
    }

    @Override
    protected void onDraw(Canvas canvas) {
        super.onDraw(canvas);

        int width = getWidth();
        int height = getHeight();

        /* Ensure our cursor doesn't escape view bounds */
        if(touchX - circleRad < 0) touchX = circleRad;
        if(touchX + circleRad > width) touchX = width - circleRad;
        if(touchY - circleRad < 0) touchY = circleRad;
        if(touchY + circleRad > height) touchY = height - circleRad;

        if(!isTouching) {
            touchX = width / 2;
            touchY = height / 2;
        }

        canvas.drawColor(backColor);
        canvas.drawCircle(touchX, touchY, circleRad, circlePainter);
    }

    @Override
    public boolean onTouchEvent(MotionEvent me ) {
        if(me.getAction() == me.ACTION_DOWN) {
            isTouching = true;
        }
        else if(me.getAction() == me.ACTION_UP) {
            isTouching = false;
        }

        touchX = me.getX();
        touchY = me.getY();

        invalidate();

        return true;
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
}
