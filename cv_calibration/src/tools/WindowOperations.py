def boxFromWindow(window):
    data = window.get_geometry()
    return {'top':int(data.y), 'left':int(data.x), 'width':int(data.width), 'height':int(data.height)}

def rvizRemoveBorders(box):
    titleBorder = 23
    outsideBorder = 1
    box['top'] += titleBorder
    box['left'] += outsideBorder
    box['width'] -= 2*outsideBorder # remove left and right border from width
    box['height'] -= titleBorder + outsideBorder # remove top and bottom border from width
    return box

def getColorStandardizedRect(box):
    ratio = float(box['width'])/box['height']
    standard_ratio = 16.0/9.0

    if standard_ratio > ratio:
        new_width = float(box['width'])
        new_height = float(box['width']) / standard_ratio
    else:
        new_width = float(box['height']) * standard_ratio
        new_height = float(box['height'])
    
    new_top = box['top'] + float(box['height'])/2.0 - float(new_height)/2.0
    new_left = box['left'] + float(box['width'])/2.0 - float(new_width)/2.0

    box['top']    = round(new_top)
    box['left']   = round(new_left)
    box['width']  = round(new_width)
    box['height'] = round(new_height)

    return box

def isolateCamera(window):
    """
    Start from a capture of the entire camera window and isolate the camera with a 16:9 ratio
    """
    return getColorStandardizedRect(rvizRemoveBorders(boxFromWindow(window)))