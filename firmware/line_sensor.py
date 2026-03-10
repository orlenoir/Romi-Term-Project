
from utime import sleep_ms


class QTR8A:
    

    def __init__(self, adc_list, settle_ms=1):
       
        if len(adc_list) != 8:
            raise ValueError("QTR8A expects exactly 8 ADC channels")

        self.adc = adc_list
        self.settle_ms = int(settle_ms)

        self.white = [0] * 8
        self.black = [4095] * 8

        self._last_centroid = None


    def read_raw(self):
       
        out = []
        for ch in self.adc:
            out.append(ch.read())
            if self.settle_ms > 0:
                sleep_ms(self.settle_ms)
        return out

    def calibrate_white(self, samples=50):

        samples = int(samples)
        acc = [0] * 8
        for _ in range(samples):
            vals = self.read_raw()
            for i in range(8):
                acc[i] += vals[i]
        self.white = [acc[i] / samples for i in range(8)]
        return self.white

    def calibrate_black(self, samples=50):

        samples = int(samples)
        acc = [0] * 8
        for _ in range(samples):
            vals = self.read_raw()
            for i in range(8):
                acc[i] += vals[i]
        self.black = [acc[i] / samples for i in range(8)]
        return self.black


    def read_norm(self):

        raw = self.read_raw()
        norm = [0.0] * 8

        for i in range(8):
            denom = (self.black[i] - self.white[i])

            if abs(denom) < 1e-9:
                x = 0.0
            else:
                x = (raw[i] - self.white[i]) / denom

            if x < 0.0:
                x = 0.0
            elif x > 1.0:
                x = 1.0

            norm[i] = x

        return norm

    def centroid(self, threshold=0.05):

        norm = self.read_norm()
        s = sum(norm)

        if s < float(threshold):
            return None

        weighted = 0.0
        for i, v in enumerate(norm):
            weighted += i * v

        c = weighted / s
        self._last_centroid = c
        return c

    def error(self, center=3.5, threshold=0.05):

        c = self.centroid(threshold=threshold)
        if c is None:
            return None
        return c - float(center)


    def snapshot(self):

        raw = self.read_raw()
        norm = self.read_norm()
        c = self.centroid()
        e = None if c is None else (c - 3.5)
        return raw, norm, c, e  