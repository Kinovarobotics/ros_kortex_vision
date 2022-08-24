from numbers import Number
import cv2 as cv
from matplotlib.ft2font import ITALIC
import rospy as ros
import tf.transformations
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


def debugData(im:cv.Mat, height:int, color, **kwargs):
    font = cv.FONT_HERSHEY_SIMPLEX # get font
    scale = cv.getFontScaleFromHeight(font, height) # get scale from pixel height
    for i, (key, val) in enumerate(kwargs.items()): # get keywords from params plus index
        pos = (10,10 + (height + 10) * (i + 1)) # position of bottom left corner of text
        text = f'{key}: {val}' # format the keyvalue to <key>: <val>
        cv.putText(im, text, pos, font, scale, color, 2, cv.LINE_AA) # write text to image

class InputOption:
    """
    Used for checkInput to use as options to display to the user
    """
    def __init__(self, optionName:str, optionDesc:str):
        """
        optionName: The str to match in the input\n
        optionDesc: A description of the action
        """
        self.name = optionName
        self.desc = optionDesc

    def __str__(self):
        return f'\t({self.name}): {self.desc}'

def checkInput(prompt:object='',*options:InputOption, logType=ros.loginfo, acceptEmpty=None, emptyDesc='Continue'):
    """
    Prompts the user to enter an option. It also lists the options.\n
    Returns the text sent by the user
    """

    # Create the option to quit
    optQuit = InputOption('q', 'Exit the program')
    p = f'{prompt}'
    p += f'\n{optQuit}'

    # Add the additional options
    for opt in options:
        p += f'\n{opt}'

    # If there are no additional options, by default an empty message is valid. However if acceptEmpty is set to False, 
    # an empty message will fail and prompt the user again
    if (acceptEmpty is None and len(options) == 0) or acceptEmpty is True:
        optContinue = InputOption('Press [Enter]', emptyDesc)
        p += f'\n{optContinue}'

    # Promt the user until a valid option is chosen
    while True:
        logType(p)
        try:
            c = input()
        except EOFError: # ctrl+d will end the program
            raise KeyboardInterrupt()
        if c == 'q': # End the program
            raise KeyboardInterrupt()
        if (acceptEmpty is None and c == '' and len(options) == 0) or acceptEmpty is True: # Manage empty input
            break
        if c in [opt.name for opt in options]: # Check if valid option
            break
        ros.logerr(f'Invalid option selected: {(c if c != "" else "[Enter]")}') # Notify if invalid option
    
    return c

def sendTransform(x:float, y:float, z:float, bc:StaticTransformBroadcaster):
    """
    Send the depth transform
    """
    tfstamped = TransformStamped()
    tfstamped.header.stamp = ros.Time.now()
    tfstamped.header.frame_id = 'camera_link'
    tfstamped.child_frame_id = 'camera_depth_frame'
    tfstamped.transform.translation.x = x
    tfstamped.transform.translation.y = y
    tfstamped.transform.translation.z = z
    quat = tf.transformations.quaternion_from_euler(0,0,0)
    tfstamped.transform.rotation.x = quat[0]
    tfstamped.transform.rotation.y = quat[1]
    tfstamped.transform.rotation.z = quat[2]
    tfstamped.transform.rotation.w = quat[3]

    bc.sendTransform(tfstamped)

def scale(value:Number, initMin:Number, initMax:Number, resMin:Number, resMax:Number):
    initDelta = initMax - initMin
    resDelta = resMax - resMin
    return resMin + ((value - initMin) / initDelta) * resDelta
