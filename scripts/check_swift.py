#!/usr/bin/env python3
"""There is no Swift compiler in this environment, so this only catches typos: balanced {} () [] per file,
ignoring strings and comments. It is NOT a substitute for building in Xcode."""
import glob, pathlib, sys
root = pathlib.Path(__file__).resolve().parent.parent
def strip(src):
    out, i, n = [], 0, len(src)
    while i < n:
        if src.startswith("//", i):
            j = src.find("\n", i); i = n if j < 0 else j; continue
        if src.startswith("/*", i):
            j = src.find("*/", i); i = n if j < 0 else j + 2; continue
        if src[i] == '"':
            j = i + 1
            while j < n and src[j] != '"': j += 2 if src[j] == "\\" else 1
            i = j + 1; continue
        out.append(src[i]); i += 1
    return "".join(out)
bad = 0
files = sorted(glob.glob(str(root / "ios/SummonDesk/*.swift")))
for f in files:
    s = strip(open(f).read())
    for a, b in ("{}", "()", "[]"):
        if s.count(a) != s.count(b):
            bad += 1; print("UNBALANCED", pathlib.Path(f).name, a, b, s.count(a), s.count(b))
print(f"{len(files)} Swift files, " + ("all delimiters balanced" if not bad else f"{bad} problems"))
sys.exit(1 if bad else 0)
