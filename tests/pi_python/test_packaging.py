"""Bug 6 (requirements) + modules import without rplidar/ntcore installed."""
import os
import subprocess
import sys
import unittest

import fakes


def reqs(path):
    out = []
    with open(path) as f:
        for line in f:
            line = line.split("#", 1)[0].strip()
            if line:
                out.append(line)
    return out


class TestRequirements(unittest.TestCase):
    def check(self, path, extra=()):
        r = reqs(path)
        self.assertIn("rplidar-roboticia==0.9.5", r)
        self.assertIn("pyserial>=3.5", r)
        for bad in ("rplidar>=1.0.0", "networktables>=3.0.0"):
            self.assertNotIn(bad, r)
        names = [x.split("=")[0].split(">")[0] for x in r]
        self.assertNotIn("rplidar", names)      # conflicts with the fork
        self.assertNotIn("networktables", names)
        for e in extra:
            self.assertTrue(any(x.startswith(e + "==") for x in r), (e, r))

    def test_pi_requirements(self):
        self.check(os.path.join(fakes.PI_SRC, "requirements.txt"), ["pyntcore"])

    def test_lidar_tool_requirements(self):
        self.check(os.path.join(fakes.LIDAR_TOOL_SRC, "requirements.txt"))


class TestImportWithoutDrivers(unittest.TestCase):
    """Real packages are NOT installed here; imports must still succeed."""

    def run_py(self, src_dir, code):
        return subprocess.run(
            [sys.executable, "-I", "-c",
             "import sys; sys.path.insert(0, %r); %s" % (src_dir, code)],
            capture_output=True, text=True, timeout=30)

    def test_pi_modules_import_without_rplidar_or_ntcore(self):
        p = self.run_py(fakes.PI_SRC,
                        "import lidar_reader, nt_client, safety_monitor")
        self.assertEqual(p.returncode, 0, p.stdout + p.stderr)

    def test_lidar_tool_imports_and_reports_metres(self):
        p = self.run_py(fakes.LIDAR_TOOL_SRC,
                        "import lidar_scan_test as t; "
                        "v,_=t.summarize([(15,10.0,500.0),(0,20.0,0)]); "
                        "print(t.nearest_m(v))")
        self.assertEqual(p.returncode, 0, p.stdout + p.stderr)
        self.assertEqual(p.stdout.strip(), "[(10.0, 0.5)]")


if __name__ == "__main__":
    unittest.main()
