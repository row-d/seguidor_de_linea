#!/usr/bin/env python3
"""Host checks; no Arduino, external libraries or hardware required."""
from pathlib import Path
import shutil
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
compiler = shutil.which("c++")
if not compiler:
    raise SystemExit("A C++ compiler is required.")
with tempfile.TemporaryDirectory(prefix="line-tests-") as temporary:
    binary = str(Path(temporary) / "line-tests")
    subprocess.run([
        compiler, "-std=c++11", "-Wall", "-Wextra", "-Werror", "-pedantic",
        "-I" + str(root / "tests/stubs"),
        "-I" + str(root / "components/src"),
        str(root / "tests/line_control_test.cpp"), "-o", binary,
    ], check=True)
    subprocess.run([binary], check=True)
