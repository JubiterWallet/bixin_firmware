#!/usr/bin/env python3
import os
import sys

#FTFixed: 增加了SE升级之后，32K空间不足，增加为64K
TOTALSIZE = 48 * 1024
MAXSIZE = TOTALSIZE - 32

infile = sys.argv[1]
outfile = sys.argv[2]
fs = os.stat(infile).st_size
print(f"Current bootloader size before align is {fs} bytes")
if fs > MAXSIZE:
    raise Exception(
        f"bootloader has to be smaller than {MAXSIZE} bytes (current size is {fs})"
    )
with open(outfile, "wb") as f:
    with open(infile, "rb") as i:
        f.write(i.read())
    f.write(b"\x00" * (TOTALSIZE - fs))
