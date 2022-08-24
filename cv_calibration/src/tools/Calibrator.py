class Calibrator():
    @property
    def error(self):
        return self.currentVal

    def __init__(self, init_tuning:float):
        self.init_tuning = self.crawlingSpeed = init_tuning
        self.currentVal = 0
        self.lastVal = 0
        self.output = 0

    def reset(self):
        self.crawlingSpeed = self.init_tuning
        self.currentVal = 0
        self.lastVal = 0
        self.output = 0

    def compute(self, currentVal:float):
        """
        @brief Compute one iteration of the calibration algorithm
        @param currentVal new process value
        """
        self.lastVal = self.currentVal # Update last value
        self.currentVal = currentVal # Update process value

        if (self.lastVal > 0) == (self.currentVal > 0): # Has not overshot
            # If last attempt made the error bigger, we are going the wrong way, flip the crawling direction
            self.crawlingSpeed = self.crawlingSpeed if abs(self.lastVal) - abs(self.currentVal) > 0 else -self.crawlingSpeed
        else: # Overshot
            self.crawlingSpeed *= -0.8 # Reverse and reduce crawling speed
        
        self.output += self.crawlingSpeed # Update output