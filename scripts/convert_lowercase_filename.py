import os
import sys


if len(sys.argv) != 2:
    print("Usage: python convert_lowercase_filename.py <string>")
    sys.exit(1)

s = sys.argv[1]
converted_s = ""

for ch in s:
    if ch in ":~!@#$%^&*(),. ":
        converted_s += '_'
    else:
        converted_s += ch

print(converted_s.lower())

