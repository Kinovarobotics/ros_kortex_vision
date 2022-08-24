import enum
from math import sqrt
from numbers import Number
import random
from typing import Any, Callable, Iterable, List, Tuple, Union, overload

import cv2 as cv
import numpy as np
from tools.utils import debugData, scale
from tools.ValueFilter import ValueFilter
import rospy as ros

class rectFloat:
    def __init__(self):
        self.x = float(0)
        self.y = float(0)
        self.w = float(0)
        self.h = float(0)

    def toAbsolute(self, screenSize:Tuple[int,int]):
        res = rectInt()
        res.x = int(self.x * screenSize[0])
        res.w = int(self.w * screenSize[0])
        res.y = int(self.y * screenSize[1])
        res.h = int(self.h * screenSize[1])
        return res

class rectInt:
    def __init__(self, obj:Union[Tuple[int,int,int,int],List[int],Any]=None, x:int=None, y:int=None, w:int=None, h:int=None):
        self.x = int(0)
        self.y = int(0)
        self.w = int(0)
        self.h = int(0)
        if obj is not None:
            if hasattr(obj, 'x') and hasattr(obj, 'y') and hasattr(obj, 'w') and hasattr(obj, 'h'):
                self.x = int(obj.x)
                self.y = int(obj.y)
                self.w = int(obj.w)
                self.h = int(obj.h)
                return
            if hasattr(obj, '__getitem__') and len(obj) >= 4:
                self.x = obj[0]
                self.y = obj[1]
                self.w = obj[2]
                self.h = obj[3]
                return
        if x is not None:
            self.x = x
        if y is not None:
            self.y = y
        if w is not None:
            self.w = w
        if h is not None:
            self.h = h

    def toRelative(self, screenSize:Tuple[int,int]):
        res = rectFloat()
        res.x = float(self.x) / screenSize[0]
        res.w = float(self.w) / screenSize[0]
        res.y = float(self.y) / screenSize[1]
        res.h = float(self.h) / screenSize[1]
        return res

class processingMethod(enum.Enum):
    THRESH_METHOD = 0
    DEPTH_THRESH_METHOD = 1

class adjustParam:
    def __init__(self, initVal: Number, increaseKey:int, decreaseKey:int, step:Number = 5, range:Tuple[Number,Number]=None):
        self.value = initVal
        self.increaseKey = increaseKey
        self.decreaseKey = decreaseKey
        self.step = step
        self.range = range

    def update(self, key:int):
        if key & 0xFF == self.increaseKey:
            self.value+=self.step
        if key & 0xFF == self.decreaseKey:
            self.value-=self.step
        if self.range is not None:
            self.value = min(self.range[1], max(self.range[0], self.value))



    def __str__(self):
        return f'Up:`{chr(self.increaseKey)}` Down:`{chr(self.decreaseKey)}` Value:{self.value}'

class screenImageProcessor:
    def __init__(self,name:str, encoding:str, method:processingMethod, **kwargs:adjustParam):
        """
        For method == calibrationMethod.THRESH_METHOD, args:\n
            -thresh\n
        For method == calibrationMethod.DEPTH_THRESH_METHOD, args:\n
            -thresh\n
        """
        self.name = name
        self.rect = rectInt()
        self.method = method
        self.calibArgs = kwargs
        self.encoding = encoding
        self.isInit = False
        self.filter = ValueFilter(2, 1, 10, 4)
        self.depthRange = (255, 0)

    def init(self):
        if not self.isInit:
            cv.namedWindow(self.name, cv.WINDOW_AUTOSIZE)
            cv.namedWindow(f'{self.name}_canny', cv.WINDOW_AUTOSIZE)
            cv.namedWindow(f'{self.name}_filter', cv.WINDOW_AUTOSIZE)
            self.isInit = True

    def findBoundingBoxes(self, canny:cv.Mat):
        cnts = cv.findContours(canny, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[-2:]
        cnts = cnts[0] if len(cnts) == 2 else cnts[1]
        for c in cnts:
            x, y, w, h = cv.boundingRect(c)
            yield rectInt(x=x, y=y, w=w, h=h)

    def findBiggestBox(self, boxes:Iterable[rectInt]):
        x, y, w, h = (0,0,0,0)
        area = 0
        for xx, yy, ww, hh in ((rect.x, rect.y, rect.w, rect.h) for rect in boxes):
            if ww * hh > area:
                if area != 0: # Don't yield a 0,0,0,0 box
                    # Yield the previously biggest box since it was never yielded
                    yield rectInt(x=x, y=y, w=w, h=h)

                x = xx
                y = yy
                w = ww
                h = hh
                area = w * h
                # To avoid yielding the biggest box multiple times, we don't yield here
            else: # Yield every box that isn't the biggest
                yield rectInt(x=xx, y=yy, w=ww, h=hh) # Yield to streamline the process only iterating through once
        yield rectInt(x=x, y=y, w=w, h=h) # Last is the result
        
            
    def filterMinMaxSize(self, boxes:Iterable[rectInt], minCoverage:float, maxCoverage:float, screenArea:int):
        for x, y, w, h in ((rect.x, rect.y, rect.w, rect.h) for rect in boxes):
            coverage = (w * h) / screenArea
            if minCoverage < coverage and coverage < maxCoverage:
                yield rectInt(x=x, y=y, w=w, h=h) # Yield to streamline the process only iterating through once

    def findFillRatio(self, box:rectInt, thresh:cv.Mat, whiteFill=True):
        if whiteFill:
            return np.average(thresh[box.y:box.y+box.h, box.x:box.x+box.w]) / 255
        else:
            return 1 - (np.average(thresh[box.y:box.y+box.h, box.x:box.x+box.w]) / 255)

    def filterThreshFillRatio(self, boxes:Iterable[rectInt], thresh:cv.Mat, minFilling:float=0.9, whiteFill=True):
        for x, y, w, h in ((rect.x, rect.y, rect.w, rect.h) for rect in boxes):
            if self.findFillRatio(rectInt((x,y,w,h)), thresh, whiteFill) > minFilling:
                yield rectInt(x=x, y=y, w=w, h=h)

    def filterMargin(self, boxes:Iterable[rectInt], relativeMargin:float, width:int, height:int):
        for rect in boxes:
            relRect = rect.toRelative((width, height))
            if (relativeMargin < relRect.x) and (relRect.x + relRect.w < 1.0 - relativeMargin) and (relativeMargin < relRect.y) and (relRect.y + relRect.h < 1.0 - relativeMargin):
                yield rect

    def relDistanceToMid(self, box:rectInt, screenSize:Tuple[int,int]):
        relBox = box.toRelative((screenSize[1],screenSize[0]))
        return np.linalg.norm(np.array([relBox.x + 0.5, relBox.y + 0.5]) - np.array([0.5,0.5]))

    def calcDistSizeScore(self, relDist:float, relSize:float, ratio:float):
        """
        ratio is the ratio of impact of size compared to distance.
        A ratio of 1 means that the distance is ignored.
        If 0.8 is used, then the size will have an impact of 0.8 and the distance will have an impact of 0.2
        """
        return relSize * ratio + (1.0 - ratio) * (1 - relDist / sqrt(2))

    def findBestScore(self, boxes:Iterable[rectInt], ratio:float, screenSize:Tuple[int,int]):
        x, y, w, h = (0,0,0,0)
        score = 0
        for xx, yy, ww, hh, rect in ((rect.x, rect.y, rect.w, rect.h, rect) for rect in boxes):
            area = (ww * hh) / (screenSize[0]*screenSize[1])
            dist = self.relDistanceToMid(rect, screenSize)
            s = self.calcDistSizeScore(dist, area, ratio)

            if s > score:
                if x==0 and y==0 and w==0 and h==0: # Don't yield a 0,0,0,0 box
                    # Yield the previously best box since it was never yielded
                    yield rectInt(x=x, y=y, w=w, h=h)

                x = xx
                y = yy
                w = ww
                h = hh
                score = w * h
                # To avoid yielding the best box multiple times, we don't yield here
            else: # Yield every box that isn't the best
                yield rectInt(x=xx, y=yy, w=ww, h=hh) # Yield to streamline the process only iterating through once
        yield rectInt(x=x, y=y, w=w, h=h) # Last is the result

    def drawBoxes(self, boxes:Iterable[rectInt], img:cv.Mat):
        for x, y, w, h, rect in ((rect.x, rect.y, rect.w, rect.h, rect) for rect in boxes):
            color = (random.randrange(0, 255), random.randrange(0, 255), 127)
            cv.rectangle(img, (x,y), (x+w, y+h), color, 1)
            cv.circle(img,(x+w,y),      5, color, 1)
            cv.circle(img,(x,y),        5, color, 1)
            cv.circle(img,(x+w,y+h),    5, color, 1)
            cv.circle(img,(x,y+h),      5, color, 1)
            yield rect

    def threshMethod(self, img:cv.Mat):
        threshVal = self.calibArgs['thresh'].value
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

        _, thresh = cv.threshold(gray, threshVal, 255, cv.THRESH_BINARY_INV)
        # thresh = cv.inRange(gray, max(0, threshVal - threshIntervalVal), threshVal)
        cv.imshow(f'{self.name}_filter', thresh)

        # Canny Edge detection
        canny = cv.Canny(thresh, 50, 200)
        cv.imshow(f'{self.name}_canny', canny)

        # Find contours
        boxes = self.findBoundingBoxes(canny)
        sized = self.filterMinMaxSize(boxes, 0.05, 0.45, img.shape[0]*img.shape[1])
        margined = self.filterMargin(sized, 0.05, img.shape[1], img.shape[0])
        drawn = self.drawBoxes(margined, img)
        filled = self.filterThreshFillRatio(drawn, thresh, 0.8, False)
        # biggest = self.findBiggestBox(filled) # This still returns every box, but the last element is the biggest box
        best = self.findBestScore(filled, 0.5, img.shape)
        # This is so that the entire process stays in generators so that iteration happens only once

        # Iterate over all boxes
        for x, y, w, h in ((rect.x, rect.y, rect.w, rect.h) for rect in best):
            pass

        fillPercent=0

        # Filter values
        if x==0 and y==0 and w==0 and h==0:
            # Show last box
            x = self.rect.x
            y = self.rect.y
            w = self.rect.w
            h = self.rect.h
            cv.rectangle(img, (x,y), (x+w, y+h),    (0,255,255), 2)
            cv.circle(img,(x,y),        10,         (0,255,255), 5)
            cv.circle(img,(x+w,y),      10,         (0,255,255), 5)
            cv.circle(img,(x+w,y+h),    10,         (0,255,255), 5)
            cv.circle(img,(x,y+h),      10,         (0,255,255), 5)
        else:
            # self.filter.write(x, y, w, h)
            # x, y, w, h = tuple([int(round(v)) for v in self.filter.value])
            fillPercent = self.findFillRatio(rectInt((x,y,w,h)), thresh, True)

            # Last box is best match
            cv.rectangle(img, (x,y), (x+w, y+h), (255,255,0), 2)
            cv.circle(img,(x,y),        10, (255,255,0), 5)
            cv.circle(img,(x+w,y),      10, (255,255,0), 5)
            cv.circle(img,(x+w,y+h),    10, (255,255,0), 5)
            cv.circle(img,(x,y+h),      10, (255,255,0), 5)
            self.rect.x = x
            self.rect.y = y
            self.rect.w = w
            self.rect.h = h

        debugData(img, 20, (255,0,0),
            TopLeft=    (x,y),
            TopRight=   (x+w,y),
            BottomRight=(x+w,y+h),
            BottomLeft= (x,y+h),
            FillPercent=fillPercent,
            **self.calibArgs)

        cv.imshow(self.name,img)

    def testMargined(self):
        boxes = [rectInt(0,0, 10, 10), rectInt(90, 90, 20, 20)]
        res = list(self.filterMargin(boxes, 0.01, 200, 200))
        assert len(res) == 1
        ros.loginfo(f'Margin test:{res}')

    def depthThreshMethod(self, img:cv.Mat):
        threshVal = self.calibArgs['thresh'].value
        hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        hue = np.zeros((img.shape[0],img.shape[1]), np.uint8)
        hue[:,:] = hsv[:,:,0]

        # img8 = scale(hue, hue.min(),hue.max(), 0, 255).astype('uint8')
        img8 = hue.astype('uint8')
        thresh = cv.inRange(img8, 1, threshVal)
        cv.imshow(f'{self.name}_filter', thresh)

        # Canny Edge detection
        canny = cv.Canny(thresh, 50, 200)
        cv.imshow(f'{self.name}_canny', canny)

        # Find contours
        boxes = self.findBoundingBoxes(canny)
        sized = self.filterMinMaxSize(boxes, 0.05, 0.45, img.shape[0]*img.shape[1])
        margined = self.filterMargin(sized, 0.05, img.shape[1], img.shape[0])
        drawn = self.drawBoxes(margined, img)
        filled = self.filterThreshFillRatio(drawn, thresh, 0.8, True)
        best = self.findBestScore(filled, 0.5, img.shape)

        # This is so that the entire process stays in generators so that iteration happens only once

        # Iterate over all boxes
        for x, y, w, h in ((rect.x, rect.y, rect.w, rect.h) for rect in best):
            pass
        

        fillPercent=0

        # Filter values
        if x==0 and y==0 and w==0 and h==0:
            # Show last box
            x = self.rect.x
            y = self.rect.y
            w = self.rect.w
            h = self.rect.h
            cv.rectangle(img, (x,y), (x+w, y+h),    (0,255,255), 2)
            cv.circle(img,(x,y),        10,         (0,255,255), 5)
            cv.circle(img,(x+w,y),      10,         (0,255,255), 5)
            cv.circle(img,(x+w,y+h),    10,         (0,255,255), 5)
            cv.circle(img,(x,y+h),      10,         (0,255,255), 5)
        else:
            # self.filter.write(x, y, w, h)
            # x, y, w, h = tuple([int(round(v)) for v in self.filter.value])
            fillPercent = self.findFillRatio(rectInt((x,y,w,h)), thresh, True)

            # Last box is best match
            cv.rectangle(img, (x,y), (x+w, y+h), (255,255,0), 2)
            cv.circle(img,(x,y),        10, (255,255,0), 5)
            cv.circle(img,(x+w,y),      10, (255,255,0), 5)
            cv.circle(img,(x+w,y+h),    10, (255,255,0), 5)
            cv.circle(img,(x,y+h),      10, (255,255,0), 5)
            self.rect.x = x
            self.rect.y = y
            self.rect.w = w
            self.rect.h = h
        
        debugData(img, 20, (255,0,0),
            TopLeft=    (x,y),
            TopRight=   (x+w,y),
            BottomRight=(x+w,y+h),
            BottomLeft= (x,y+h),
            FillPercent=fillPercent,
            **self.calibArgs)

        cv.imshow(self.name,img)

    def updateKeyPress(self, key:int):
        for k, arg in self.calibArgs.items():
            arg.update(key)

    def destroyWindows(self):
        cv.destroyWindow(self.name)
        cv.destroyWindow(f'{self.name}_filter')
        cv.destroyWindow(f'{self.name}_canny')
        self.isInit = False

    def processImage(self, input: cv.Mat):
        self.init()
        if self.method == processingMethod.THRESH_METHOD:
            self.threshMethod(input)
        if self.method == processingMethod.DEPTH_THRESH_METHOD:
            self.depthThreshMethod(input)
