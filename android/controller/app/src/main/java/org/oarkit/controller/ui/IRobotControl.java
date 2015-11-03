/*
 * OARK Controller Software.
 *
 * Copyright (c) 2015 Open Academic Robot Kit.
 */

package org.oarkit.controller.ui;

/**
 * Interface for robot controls.
 *
 * For controlling the robot, an array of float values are sent across
 * the network. Each controller type may have a different number of
 * floats that it needs to send.
 *
 * In order to make data queries consistent for classes that may be
 * used to control the robot, they must have this interface.
 *
 * @todo Come up with a way to allow more than one type to be returned
 *       but still have the interface general enough.
 */
public interface IRobotControl {
    float[] getValues();
}
