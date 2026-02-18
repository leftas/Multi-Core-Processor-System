#!/usr/bin/env python3

from trace_lib import Trace
t = Trace(__file__.replace('.py', '.trf'), 1)

t.write(0x000)   # write miss
t.write(0x200)   # write miss
t.write(0x400)
t.write(0x600)
t.write(0x800)
t.write(0xA00)
t.write(0xC00)
t.write(0xE00)
t.write(0x000)   # write miss
t.write(0x200)   # write miss
t.write(0x400)
t.write(0x600)
t.write(0x800)
t.write(0xA00)
t.write(0xC00)
t.write(0xE00)
t.write(0x1000)  # this is where evict should happen

t.close()
