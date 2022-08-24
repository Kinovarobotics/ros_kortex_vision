# Ros imports
from time import sleep
import rospy as ros
import roslaunch
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

# Image related imports
import cv2 as cv

# Tools imports
from tools.imageProcessor import screenImageProcessor, adjustParam, processingMethod, rectInt
from tools.Calibrator import Calibrator
from tools.utils import debugData, checkInput, InputOption, sendTransform
from tools.screenCapture import screenCapture
from tools.UserHelp import askRVIZ, checkTopic, findWindows
from tools.ValueFilter import ValueFilter
from tools.WindowOperations import isolateCamera

# Other imports
from random import random
from typing import Dict, Tuple
import numpy as np
import pathlib

def calibrate(xScore:float, yScore:float, scaleScore:float, xCalibrate:Calibrator, yCalibrate:Calibrator, zCalibrate:Calibrator):
    """
    Compute the calibrators
    """
    xCalibrate.compute(xScore)
    yCalibrate.compute(yScore)
    zCalibrate.compute(scaleScore)

def runCalibration(colorRect:rectInt, depthRect:rectInt, colorSize:Tuple[int,int], depthSize:Tuple[int,int], calib:bool, vFilter:ValueFilter, xCalibrate:Calibrator, yCalibrate:Calibrator, zCalibrate:Calibrator):
    """
    Run the calibration algorithm
    """
    # Transpose the rectangles from absolute (pixel) to relative (0.0-1.0) values
    colorRelRect = colorRect.toRelative(colorSize)
    depthRelRect = depthRect.toRelative(depthSize)

    # Calculate the scores
    xScore = (colorRelRect.x + colorRelRect.w/2) - (depthRelRect.x + depthRelRect.w/2)
    yScore = (colorRelRect.y + colorRelRect.h/2) - (depthRelRect.y + depthRelRect.h/2)
    scaleScore = ((colorRelRect.h - depthRelRect.h) + (colorRelRect.w - depthRelRect.w)) / 2
    
    # Filter scores
    vFilter.write(xScore, yScore, scaleScore)
    xScore, yScore, scaleScore = vFilter.value

    if calib: # Calibrate
        calibrate(xScore, yScore, scaleScore, xCalibrate, yCalibrate, zCalibrate)
    
    return xScore, yScore, scaleScore



def convertBoxToRectInt(box:Dict[str,float]):
    """
    Simple conversion from a window geometry to a rectInt
    """
    res = rectInt()
    res.x = box['left']
    res.y = box['top']
    res.w = box['width']
    res.h = box['height']
    return res

def saveData(xOutput:float, yOutput:float, zOutput:float):
    """
    Save the programed values
    """
    # Get directory paths
    resources_folder = pathlib.Path(__file__).parent.parent.joinpath('resources').resolve()
    result_file = resources_folder.joinpath("Results.txt").resolve()

    ros.loginfo(f'Saving values to: {result_file}')
    with open(result_file, 'w') as writer:
        writer.write('OUTPUT VALUES\n')
        writer.write(f'x y z: {xOutput} {yOutput} {zOutput}\n\n')
        
        writer.write('PARAMETERS\n')

        writer.write('\nCOLOR\n')
        for key, val in colorProcessor.calibArgs.items():
            writer.write(f'{key}: {val.value}\n')

        writer.write('\nDEPTH\n')
        for key, val in depthProcessor.calibArgs.items():
            writer.write(f'{key}: {val.value}\n')
        writer.close()
        
def afterCalibration(xOutput:float, yOutput:float, zOutput:float):
    """
    Code to run after calibration is stopped
    """
    saveData(xOutput, yOutput, zOutput)
    checkInput('Calibration paused.', InputOption('c', 'Resume calibration'))

if __name__ == '__main__':
    # Start node
    ros.init_node('rviz_calibration', anonymous=True)

    try:
        # Guide the user
        ros.loginfo('At any time, write "q" and press enter to exit the program or press "ctrl+d" to force end of an input')

        # Check for the camera driver to be running
        checkTopic('/camera', 'Please start the the ros_kortex_vision driver', 'Found the camera driver')

        # Init depth transform publisher
        tfbc = StaticTransformBroadcaster()

        launch_folder = pathlib.Path(__file__).parent.parent.joinpath('launch').resolve()
        # color_launch_file = launch_folder.joinpath('color_tf.launch').resolve()

        # Start color transform in parallel
        # uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        # roslaunch.configure_logging(uuid)
        # tracking_launch = roslaunch.parent.ROSLaunchParent(
        #     uuid, [str(color_launch_file)])
        # tracking_launch.start()

        # Send an initial transform to allow rviz to enable the point cloud
        sendTransform(random()*0.01,random()*0.01,random()*0.01, tfbc)

        # Guide the user to setup rviz
        askRVIZ()

        # Make the user locate the proper windows
        color, depth = findWindows()

        # Initialiaze captures
        colorCap = screenCapture(isolateCamera(color))
        depthCap = screenCapture(isolateCamera(depth))

        # Initialize the processors
        colorProcessor = screenImageProcessor('color', 'bgr8', processingMethod.THRESH_METHOD,
            thresh=adjustParam(200, ord('w'), ord('s'), 1, (0,255)))
        depthProcessor = screenImageProcessor('depth', 'bgr8', processingMethod.DEPTH_THRESH_METHOD,
            thresh=adjustParam(50, ord('e'), ord('d'), 1, (1,255)))

        # Make a debug window
        cv.namedWindow('Debug', cv.WINDOW_AUTOSIZE)

        # Setup the calibrators
        xCalibrate = Calibrator(0.004)
        yCalibrate = Calibrator(0.004)
        zCalibrate = Calibrator(0.004)

        # Value filter for the depth stream
        vFilter = ValueFilter(5, 1, 20, 3)

        doCalibrate = False

        # Main loop
        while True:
            

            # Update the window positions
            colorCap.box = isolateCamera(color)
            depthCap.box = isolateCamera(depth)

            # Capture the windows
            colorCap.capture()
            depthCap.capture()

            # Process the captures
            colorProcessor.processImage(colorCap.getImg())
            depthProcessor.processImage(depthCap.getImg())

            # Convert the sizes for relative scaling
            colorSize = (colorCap.box['width'], colorCap.box['height'])
            depthSize = (depthCap.box['width'], depthCap.box['height'])

            # Provide a way to exit code at any time (while the windows are active)
            keyPressed = cv.waitKey(3)
            if (keyPressed & 0xFF) == ord('q'):
                raise KeyboardInterrupt()

            if doCalibrate:
                vFilter.fill(xCalibrate.output, yCalibrate.output, zCalibrate.output)
                
            doCalibrate = False
            if (keyPressed & 0xFF) == ord('c'):
                # Run the calibration algorithm
                doCalibrate = vFilter.index == 0

                # Adjust transform
                sendTransform(xCalibrate.output, yCalibrate.output, zCalibrate.output, tfbc)
                # ros.loginfo_throttle(2000, 'Calibration complete!')
            xScore, yScore, scaleScore = runCalibration(colorProcessor.rect, depthProcessor.rect, colorSize, depthSize, doCalibrate, vFilter, xCalibrate, yCalibrate, zCalibrate)

            if (keyPressed & 0xFF) == ord('x'):
                xCalibrate.reset()
                yCalibrate.reset()
                zCalibrate.reset()
                sendTransform(0,0,0, tfbc)
                ros.loginfo('Reset calibration parameters')

            if (keyPressed & 0xFF) == ord('z'):
                colorProcessor.destroyWindows()
                depthProcessor.destroyWindows()
                afterCalibration(xCalibrate.output, yCalibrate.output, zCalibrate.output)


            # Update the adjustable parameters with the last pressed key
            colorProcessor.updateKeyPress(keyPressed)
            depthProcessor.updateKeyPress(keyPressed)
            
            # Debug data
            dbg=np.full((600,400),255, np.uint8)
            
            debugData(dbg, 12, 0,
                xError=xScore,
                xThreshold='< 0.02 is pretty good', 
                yError=yScore,
                yThreshold='< 0.02 is pretty good',
                zError=scaleScore,
                zThreshold='< 0.02 is pretty good',
                ExitProgram='Press q',
                Calibrate='Press and hold c',
                ResetCalibration='Press x',
                StopCalibration='Press z')
            cv.imshow('Debug', dbg)


    except KeyboardInterrupt: # End program
        cv.destroyAllWindows()
        ros.signal_shutdown('User stopped program')
        # tracking_launch.shutdown() # Shutdown the color transform
        quit(0)


