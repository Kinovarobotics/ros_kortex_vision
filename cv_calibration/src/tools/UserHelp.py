from tools.utils import checkInput, InputOption
import rospy as ros
import Xlib.display

def checkTopic(topic:str, failMsg:str, successMsg:str):
    """
    Check if a specific namespace exists
    """
    while True:
        topics = ros.get_published_topics(topic)
        if len(topics) == 0:
            checkInput(f'{failMsg}\nPress enter to retry',logType=ros.logerr)
        else:
            ros.loginfo(successMsg)
            return

def askRVIZ():
    """
    Guide the user to setup rviz to work with the program
    """
    userInput = checkInput('Start RVIZ.', InputOption('s', 'Skip rviz procedure.'), acceptEmpty=True)
    if userInput == 's': # Allow skipping
        return

    userInput = checkInput('Please open the configuration found at .../cv_calibration/rviz/CompareDepthColor.rviz.',
        InputOption('s', 'Skip rviz procedure.'), acceptEmpty=True)
    if userInput == 's': # Allow skipping
        return

    ros.loginfo('Please make sure both camera viewers are activated and visible without overlap.')
    ros.loginfo('This program uses screen captures to calibrate the RVIZ streams. The bigger the camera viewers, the more precise the calibration will be.')
    ros.loginfo('This is also why it is important that the windows do not overlap and are not covered by other windows.')

    checkInput('Done?') # Wait for user input

def findFocusWithName():
    """
    For 3 seconds, check if the active window has the name 'name'
    """

    # Get display
    disp = Xlib.display.Display()

    windowName = 'Camera'

    # Setup timeout
    end = ros.Time.now() + ros.Duration(secs=3)

    window = None

    while ros.Time.now() < end: # Wait until timeout
        window = disp.get_input_focus().focus # Get active window
        if window.get_wm_name() == windowName : # Check name
            return True, window

    ros.logwarn(f'Was expecting to find a window named "{windowName}" but found a window named "{window.get_wm_name()}".')
    return False, window

def findWindows():
    """
    Guide the user to find the color and depth windows
    """

    ros.loginfo('Once you are ready you will have 3 seconds after pressing enter to select the color camera viewer.')

    while True:
        checkInput('When ready, press enter and select the color camera window and wait for 3 seconds or until the window is found.')
        
        res, colorWindow = findFocusWithName() # Look for color window
        if res:
            userInput = checkInput('Found window!', InputOption('c', 'Continue'), InputOption('r', 'Retry'))
        else:
            userInput = checkInput('Continue anyway?', InputOption('c', 'Continue'), InputOption('r', 'Retry'),logType=ros.logwarn)
        if userInput == 'c':
            break
        ros.logwarn('Retrying...') # User chose retry
    
    ros.loginfo('Once you are ready you will have 3 seconds after pressing enter to select the depth camera viewer.')
    while True:
        checkInput('When ready, press enter and select the depth camera window and wait for 3 seconds or until the window is found.')

        res, depthWindow = findFocusWithName() # Look for depth window
        if res:
            userInput = checkInput('Found window!', InputOption('c', 'Continue'), InputOption('r', 'Retry'))
        else:
            userInput = checkInput('Continue anyway?', InputOption('c', 'Continue'), InputOption('r', 'Retry'),logType=ros.logwarn)
        if userInput == 'c':
            break
        ros.logwarn('Retrying...') # User chose retry
    
    return colorWindow, depthWindow