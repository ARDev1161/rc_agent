"""Regression guard: gRPC stream I/O must not hold the ROS control mutex.

This is a source-level concurrency check, not a substitute for a live gRPC
session/lease/disconnect or hardware acceptance test.
"""
from pathlib import Path
import unittest

SOURCE = (Path(__file__).resolve().parents[1] / "network/server.cpp").read_text()


class GrpcServerLocking(unittest.TestCase):
    def test_stream_reads_into_local_message_outside_lock(self):
        stream = SOURCE.split("grpc::Status grpcServer::DataStreamExchange(", 1)[1]
        self.assertIn("Controls incoming;", stream)
        self.assertIn("while (stream->Read(&incoming))", stream)
        self.assertIn("Sensors outgoing;", stream)
        self.assertIn("std::lock_guard<std::mutex> lock(muServer)", stream)
        # A short preflight lock before Read is valid; the per-message lock
        # must start only after the blocking Read has returned.
        preflight, loop = stream.split("while (stream->Read(&incoming))", 1)
        self.assertIn("std::lock_guard<std::mutex> lock(muServer)", preflight)
        self.assertIn("std::lock_guard<std::mutex> lock(muServer)", loop)
        self.assertLess(loop.index("std::lock_guard<std::mutex> lock(muServer)"),
                        loop.index("stream->Write(outgoing)"))
        self.assertLess(stream.index("}\\n        if (!stream->Write(outgoing))".replace("\\n", "\n")),
                        stream.index("stream->Write(outgoing)"))
        self.assertNotIn("stream->Read(controls_.get())", stream)
        self.assertNotIn("stream->Write(*sensors_)", stream)

    def test_uninitialized_transport_fails_closed(self):
        self.assertGreaterEqual(SOURCE.count("grpc::StatusCode::FAILED_PRECONDITION"), 2)


if __name__ == "__main__":
    unittest.main()
