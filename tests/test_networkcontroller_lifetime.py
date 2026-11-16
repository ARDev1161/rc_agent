"""Regression: detached gRPC server worker must own its bind address.

runServer returns immediately, so capturing its address_mask argument by
reference creates a dangling reference before BuildAndStart() may execute.
This is a source-level lifetime guard; ROS2 Humble CI compiles the source.
"""
from pathlib import Path
import re
import unittest

SOURCE = Path(__file__).resolve().parents[1] / "network" / "networkcontroller.cpp"

class NetworkControllerLifetimeTest(unittest.TestCase):
    def test_detached_server_thread_owns_address(self):
        source = SOURCE.read_text()
        start = source.index("int NetworkController::runServer(")
        body = source[start:]
        self.assertRegex(body, r"std::thread\s+thr\s*\(\s*\[this,\s*address_mask\]")
        self.assertIn("builder.AddListeningPort(address_mask", body)
        self.assertIn("thr.detach()", body)
        self.assertNotRegex(body, r"std::thread\s+thr\s*\(\s*\[&")

if __name__ == "__main__":
    unittest.main()
