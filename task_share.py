import array
import pyb
import micropython

share_list = []

type_code_strings = {
    'b': "int8", 'B': "uint8",
    'h': "int16", 'H': "uint16",
    'i': "int", 'I': "uint",
    'l': "int32", 'L': "uint32",
    'q': "int64", 'Q': "uint64",
    'f': "float", 'd': "double"
}

def show_all():
    return '\n'.join(str(item) for item in share_list)

class BaseShare:
    def __init__(self, type_code, thread_protect=True, name=None):
        self._type_code = type_code
        self._thread_protect = thread_protect
        self._name = name
        share_list.append(self)

class Queue(BaseShare):
    ser_num = 0

    def __init__(self, type_code, size, thread_protect=False, overwrite=False, name=None):
        super().__init__(type_code, thread_protect, name)
        self._size = size
        self._overwrite = overwrite
        self._name = name if name is not None else 'Q' + str(Queue.ser_num)
        Queue.ser_num += 1
        self._buffer = array.array(type_code, [0] * size)
        self._rd_idx = 0
        self._wr_idx = 0
        self._num_items = 0
        self._max_full = 0

    @micropython.native
    def put(self, item, in_ISR=False):
        if self.full():
            if in_ISR:
                return
            if not self._overwrite:
                while self.full():
                    pass

        if self._thread_protect and not in_ISR:
            irq_state = pyb.disable_irq()

        self._buffer[self._wr_idx] = item
        self._wr_idx += 1
        if self._wr_idx >= self._size:
            self._wr_idx = 0

        if self._num_items < self._size:
            self._num_items += 1
        if self._num_items > self._max_full:
            self._max_full = self._num_items

        if self._thread_protect and not in_ISR:
            pyb.enable_irq(irq_state)

    @micropython.native
    def get(self, in_ISR=False):
        while self.empty():
            pass

        if self._thread_protect and not in_ISR:
            irq_state = pyb.disable_irq()

        val = self._buffer[self._rd_idx]
        self._rd_idx += 1
        if self._rd_idx >= self._size:
            self._rd_idx = 0
        self._num_items -= 1

        if self._thread_protect and not in_ISR:
            pyb.enable_irq(irq_state)

        return val

    @micropython.native
    def any(self):
        return self._num_items > 0

    @micropython.native
    def empty(self):
        return self._num_items <= 0

    @micropython.native
    def full(self):
        return self._num_items >= self._size

    @micropython.native
    def num_in(self):
        return self._num_items

    def clear(self):
        self._rd_idx = 0
        self._wr_idx = 0
        self._num_items = 0
        self._max_full = 0

    def __repr__(self):
        return '{:<8s} Queue<{:s}> {:d}/{:d}'.format(
            self._name, type_code_strings[self._type_code], self._max_full, self._size
        )

class Share(BaseShare):
    ser_num = 0

    def __init__(self, type_code, thread_protect=True, name=None):
        super().__init__(type_code, thread_protect, name)
        self._buffer = array.array(type_code, [0])
        self._name = name if name is not None else 'S' + str(Share.ser_num)
        Share.ser_num += 1

    @micropython.native
    def put(self, data, in_ISR=False):
        if self._thread_protect and not in_ISR:
            irq_state = pyb.disable_irq()
        self._buffer[0] = data
        if self._thread_protect and not in_ISR:
            pyb.enable_irq(irq_state)

    @micropython.native
    def get(self, in_ISR=False):
        if self._thread_protect and not in_ISR:
            irq_state = pyb.disable_irq()
        val = self._buffer[0]
        if self._thread_protect and not in_ISR:
            pyb.enable_irq(irq_state)
        return val

    def __repr__(self):
        return '{:<8s} Share<{:s}>'.format(
            self._name, type_code_strings[self._type_code]
        )