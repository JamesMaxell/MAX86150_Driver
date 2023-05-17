# 这段程序是一个循环缓冲区的简单实现，使用了Python标准库collections中的deque类1。deque是一个双端队列，可以从两端进行操作，与Python的基本数据类型列表很相似2。
# 
# 这个程序中的CircularBuffer类有以下方法：
# 
# init(self, max_size)：初始化一个CircularBuffer对象，max_size是缓冲区的最大大小。
# len(self)：返回缓冲区中元素的数量。
# is_empty(self)：检查缓冲区是否为空。
# append(self, item)：将item添加到缓冲区的末尾。如果缓冲区已满，则删除第一个元素并将item添加到末尾。
# pop(self)：从缓冲区的开头弹出一个元素并返回它。
# clear(self)：清空缓冲区。
# pop_head(self)：从缓冲区的开头弹出一个元素并返回它。如果缓冲区中只有一个元素，则不执行任何操作。如果缓冲区中有多个元素，则删除所有元素并返回最后一个元素1。

from ucollections import deque


class CircularBuffer(object):
    ''' Very simple implementation of a circular buffer based on deque '''
    def __init__(self, max_size):
        self.data = deque((), max_size, True)
        self.max_size = max_size

    def __len__(self):
        return len(self.data)

    def is_empty(self):
        return not bool(self.data)

    def append(self, item):
        try:
            self.data.append(item)
        except IndexError:
            # deque full, popping 1st item out
            self.data.popleft()
            self.data.append(item)

    def pop(self):
        return self.data.popleft()

    def clear(self):
        self.data = deque((), self.max_size, True)

    def pop_head(self):
        buffer_size = len(self.data)
        temp = self.data
        if buffer_size == 1:
            pass
        elif buffer_size > 1:
            self.data= deque((), self.max_size, True)
            for x in range(buffer_size - 1):
                self.data = temp.popleft()
        else:
            return 0
        return temp.popleft()