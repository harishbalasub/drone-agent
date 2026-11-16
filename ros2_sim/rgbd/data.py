"""Strict timestamp matching and image decoding for Gazebo dataset capture."""
import threading
import time
import numpy as np


def stamp(message):
    return int(message.header.stamp.sec)*1000000000+int(message.header.stamp.nsec)


def decode_image(message,channels=3,dtype=np.uint8):
    width,height=int(message.width),int(message.height);item=np.dtype(dtype).itemsize
    raw=np.frombuffer(message.data,dtype=np.uint8)
    if len(raw)<int(message.step)*height:raise ValueError('Truncated image')
    rows=raw[:int(message.step)*height].reshape(height,int(message.step))[:,:width*channels*item].copy()
    return rows.reshape(-1).view(dtype).reshape(height,width,channels).squeeze(-1) if channels==1 else rows.reshape(height,width,channels)


class Synchronizer:
    def __init__(self):self.lock=threading.Lock();self.buffers={k:{} for k in ['rgb','depth','mask']}
    def add(self,kind,message):
        with self.lock:
            buffer=self.buffers[kind];buffer[stamp(message)]=message
            while len(buffer)>30:buffer.pop(min(buffer))
    def matched(self,after_ns=0):
        with self.lock:
            common=set.intersection(*(set(b) for b in self.buffers.values()))
            candidates=[t for t in common if t>after_ns]
            if not candidates:return None
            t=max(candidates);return t,{k:b[t] for k,b in self.buffers.items()}
