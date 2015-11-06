/*
 * OARK Controller Software.
 *
 * Copyright (c) 2015 Open Academic Robot Kit.
 */

package org.oarkit.controller.ui;

import android.content.Context;
import android.util.Log;

import org.oarkit.controller.messages.rosmessages.Input;

import java.util.ArrayList;
import java.util.List;

/**
 * This class is responsible for mapping the inputs to UI elements.
 *
 * The order that UI elements are presented onscreen differ from the
 * order that they get sent across the network. This class, will take
 * the input list from the robot, build the UI elements and create
 * them. It will return a list of these elements in the required order
 * for them to be sent back to the robot.
 */
public class ControllerMapping {
    private final static String SLIDER_TYPE = "SLIDER";
    private final static String STICK_TYPE = "ANALOG";

    /**
     * Given a UI context and input list, create and add the UI
     * elements to the context and return a list of IRobotControllers.
     *
     * If there are no inputs passed in, will return an empty list.
     */
    public static List<IRobotControl> mapAndCreateControllers(
        Context inUi,
        ControllerTable inTable,
        List<Input> inInputs) {

        List<IRobotControl> result = new ArrayList<>();

        /*
         * Our mapping is very simple, sliders first, then just
         * have the sticks across the bottom in order. We will
         * keep sliders and sticks in separate lists as we're
         * building, because the sliders need a span value because
         * of the table layout we're using.
         */
        List<ControllerSeekBar> sliders = new ArrayList<>();
        List<ControllerStick> sticks = new ArrayList<>();

        for (Input input : inInputs) {
            switch (input.getType().toUpperCase()) {
            case SLIDER_TYPE:
                Log.i("ControllerMapping", "Slider Title: " + input.getTitle());
                ControllerSeekBar currentSlider =
                    new ControllerSeekBar(inUi, input.getTitle());
                result.add(currentSlider);
                sliders.add(currentSlider);
                break;
            case STICK_TYPE:
                ControllerStick currentStick = new ControllerStick(inUi);
                currentStick.setTitle(input.getTitle());
                switch (input.getAxes().toUpperCase()) {
                    case "X":
                        currentStick.setAxes(ControllerStick.Axes.X);
                        break;
                    case "Y":
                        currentStick.setAxes(ControllerStick.Axes.Y);
                        break;
                    case "XY":
                    case "YX":
                        currentStick.setAxes(ControllerStick.Axes.BOTH);
                        break;
                    default:
                        Log.e("ControllerMapping", "Unknown axes demand on analog stick.");
                        currentStick.setAxes(ControllerStick.Axes.BOTH);
                        break;
                }
                result.add(currentStick);
                sticks.add(currentStick);
                break;
            default:
                break;
            }
        }

        int noOfSticks = (sticks.size() > 0) ? sticks.size() : 1;

        // Sliders first
        for (ControllerSeekBar slider : sliders) {
            slider.setSpan(noOfSticks);
            inTable.addSlider(slider);
        }

        if (sliders.size() > 0 & (sticks.size() > 1)) {
            inTable.addSpacerRow();
        }

        // Sticks
        for (ControllerStick stick : sticks) {
            inTable.addStick(stick);
        }

        return result;
    }
}
