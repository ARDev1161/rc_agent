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
        self.assertLess(stream.index("stream->Read(&incoming)"),
                        stream.index("std::lock_guard<std::mutex> lock(muServer)"))
        self.assertLess(stream.index("}\\n        if (!stream->Write(outgoing))".replace("\\n", "\n")),
                        stream.index("stream->Write(outgoing)"))
        self.assertNotIn("stream->Read(controls_.get())", stream)
        self.assertNotIn("stream->Write(*sensors_)", stream)

    def test_uninitialized_transport_fails_closed(self):
        self.assertGreaterEqual(SOURCE.count("grpc::StatusCode::FAILED_PRECONDITION"), 2)


if __name__ == "__main__":
    unittest.main()
