// Headless integration test: the real rc_agent gRPC client against a loopback
// AmurCore-compatible service. No ROS2 executor or physical robot needed.
#include "network/client.h"

#include <atomic>
#include <chrono>
#include <future>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

namespace {
void require(bool good, const char *message) {
    if (!good) throw std::runtime_error(message);
}
class OperatorMock final : public Robot::ClientOnRobot::Service {
public:
    std::atomic<bool> received{false};
    std::atomic<unsigned> stream_count{0};
    std::atomic<bool> fail_next_unary{false};
    std::atomic<bool> fail_next_stream{false};
    std::atomic<unsigned> unary_count{0};
    std::atomic<bool> hold_unary{false};
    std::atomic<bool> unary_waiting{false};
    std::atomic<bool> release_reply{false};
    std::mutex observed_mutex;
    std::string observed_robot_id;
    std::string observed_capability;
    std::string observed_axis;
    bool observed_battery{false};
    float observed_battery_percentage{0.0f};

    std::atomic<unsigned> map_snapshots{0};
    std::atomic<bool> close_next_map{false};
    std::atomic<std::uint64_t> last_map_seq{0};
    grpc::Status MapStream(
        grpc::ServerContext *ctx,
        grpc::ServerReaderWriter<map_service::GetMapRequest,
                                 map_service::GetMapResponse> *stream) override {
        map_service::GetMapResponse incoming;
        if (!stream->Read(&incoming)) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "Missing map snapshot");
        }
        last_map_seq.store(incoming.map_seq());
        ++map_snapshots;
        map_service::GetMapRequest reply;
        if (!stream->Write(reply)) {
            return grpc::Status::OK;
        }
        if (close_next_map.exchange(false)) {
            return grpc::Status::OK;  // Simulate a clean peer-side EOF.
        }
        // Leave the RPC idle after the first map snapshot. A client shutdown
        // must interrupt the stream without requiring another map update.
        while (!ctx->IsCancelled()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        return grpc::Status(grpc::StatusCode::CANCELLED, "Client stopped");
    }

    std::atomic<unsigned> pose_snapshots{0};
    std::atomic<bool> close_next_pose{false};
    std::atomic<unsigned> zone_snapshots{0};
    std::atomic<bool> close_next_zone{false};
    std::atomic<bool> accept_zone_updates{false};
    std::atomic<std::uint64_t> last_zone_id{0};
    std::atomic<int> last_zone_count{-1};

    grpc::Status PoseStream(
        grpc::ServerContext *ctx,
        grpc::ServerReaderWriter<map_service::PoseRequest,
                                 map_service::PoseState> *stream) override {
        map_service::PoseState incoming;
        if (!stream->Read(&incoming)) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "Missing pose snapshot");
        }
        ++pose_snapshots;
        map_service::PoseRequest reply;
        if (!stream->Write(reply)) return grpc::Status::OK;
        if (close_next_pose.exchange(false)) return grpc::Status::OK;
        while (!ctx->IsCancelled()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        return grpc::Status(grpc::StatusCode::CANCELLED, "Client stopped");
    }

    grpc::Status ZoneStream(
        grpc::ServerContext *ctx,
        grpc::ServerReaderWriter<map_service::ZoneRequest,
                                 map_service::ZoneState> *stream) override {
        map_service::ZoneState incoming;
        if (!stream->Read(&incoming)) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "Missing zone snapshot");
        }
        last_zone_count.store(incoming.zone_map().zones_size());
        last_zone_id.store(incoming.zone_map().zones_size() > 0
                               ? incoming.zone_map().zones(0).id() : 0);
        ++zone_snapshots;
        map_service::ZoneRequest reply;
        if (!stream->Write(reply)) return grpc::Status::OK;
        if (close_next_zone.exchange(false)) return grpc::Status::OK;
        if (accept_zone_updates.load()) {
            // Exercise changes on the SAME live RPC, not only fresh snapshots.
            while (stream->Read(&incoming)) {
                last_zone_count.store(incoming.zone_map().zones_size());
                last_zone_id.store(incoming.zone_map().zones_size() > 0
                                       ? incoming.zone_map().zones(0).id() : 0);
                ++zone_snapshots;
                if (!stream->Write(reply)) break;
            }
            return grpc::Status(grpc::StatusCode::CANCELLED, "Client stopped");
        }
        while (!ctx->IsCancelled()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        return grpc::Status(grpc::StatusCode::CANCELLED, "Client stopped");
    }

    void capture(const Robot::Sensors &incoming) {
        std::lock_guard<std::mutex> lock(observed_mutex);
        observed_robot_id = incoming.robot_info().robot_id();
        observed_battery = incoming.has_batterystate();
        observed_battery_percentage = incoming.batterystate().percentage();
        observed_capability.clear();
        observed_axis.clear();
        if (incoming.robot_info().capabilities_size() > 0) {
            const auto &cap = incoming.robot_info().capabilities(0);
            observed_capability = cap.id();
            if (cap.velocity_axes_size() > 0) {
                observed_axis = cap.velocity_axes(0);
            }
        }
    }
    bool matchesDescriptor() {
        std::lock_guard<std::mutex> lock(observed_mutex);
        return observed_robot_id == "robot-test" &&
               observed_capability == "manual.velocity" &&
               observed_axis == "linear.x";
    }

    grpc::Status DataExchange(grpc::ServerContext *ctx,
                               const Robot::Sensors *incoming,
                               Robot::Controls *reply) override {
        ++unary_count;
        if (hold_unary.load()) {
            unary_waiting.store(true);
            while (hold_unary.load() && !ctx->IsCancelled()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
            if (ctx->IsCancelled()) {
                return grpc::Status(grpc::StatusCode::CANCELLED, "Unary canceled");
            }
        }
        if (fail_next_unary.exchange(false)) {
            return grpc::Status(grpc::StatusCode::UNAVAILABLE, "Simulated disconnect");
        }
        capture(*incoming);
        reply->mutable_basecontrol()->set_linearvelocity(11.0);
        return grpc::Status::OK;
    }

    grpc::Status DataStreamExchange(
        grpc::ServerContext *ctx,
        grpc::ServerReaderWriter<Robot::Controls, Robot::Sensors> *stream) override {
        Robot::Sensors incoming;
        if (!stream->Read(&incoming)) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "Missing telemetry");
        }
        if (fail_next_stream.exchange(false)) {
            return grpc::Status(grpc::StatusCode::UNAVAILABLE, "Simulated stream disconnect");
        }
        capture(incoming);
        received = true;
        ++stream_count;
        while (!release_reply.load() && !ctx->IsCancelled()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        if (ctx->IsCancelled()) {
            return grpc::Status(grpc::StatusCode::CANCELLED, "Client canceled");
        }
        Robot::Controls reply;
        reply.mutable_basecontrol()->set_linearvelocity(11.0);
        stream->Write(reply);
        return grpc::Status::OK;
    }
};
}  // namespace

int main() {
    OperatorMock mock;
    grpc::ServerBuilder builder;
    int port = 0;
    builder.AddListeningPort("127.0.0.1:0", grpc::InsecureServerCredentials(), &port);
    builder.RegisterService(&mock);
    auto server = builder.BuildAndStart();
    require(server != nullptr && port != 0, "Could not start operator mock");

    const auto address = "127.0.0.1:" + std::to_string(port);
    auto channel = grpc::CreateChannel(address, grpc::InsecureChannelCredentials());
    require(channel->WaitForConnected(std::chrono::system_clock::now() +
                                      std::chrono::seconds(3)),
            "Could not connect to loopback service");

    auto controls = std::make_shared<Robot::Controls>();
    auto sensors = std::make_shared<Robot::Sensors>();
    auto map = std::make_shared<map_service::GetMapResponse>();
    sensors->mutable_robot_info()->set_robot_id("robot-test");
    sensors->mutable_robot_info()->set_api_major(2);
    auto *cap = sensors->mutable_robot_info()->add_capabilities();
    cap->set_id("manual.velocity");
    cap->add_velocity_axes("linear.x");
    grpcClient client(channel, controls, sensors, map, false);

    try {
        require(client.DataExchange().ok(), "Legacy-compatible unary RPC failed");
        require(mock.matchesDescriptor(), "RobotInfo was not delivered");
        {
            auto mutex = client.getMutex();
            std::lock_guard<std::mutex> lock(*mutex);
            require(controls->basecontrol().linearvelocity() == 11.0,
                    "Unary controls reply was not applied");
        }

        // Battery telemetry uses the existing optional Sensors field. Missing
        // data must remain absent, not a fabricated 0% battery.
        {
            std::lock_guard<std::mutex> lock(mock.observed_mutex);
            require(!mock.observed_battery,
                    "Robot without battery incorrectly advertised a battery");
        }
        {
            std::lock_guard<std::mutex> lock(*client.getMutex());
            sensors->mutable_batterystate()->set_percentage(0.625f);
            sensors->mutable_batterystate()->set_voltage(24.1f);
            sensors->mutable_batterystate()->set_present(true);
        }
        require(client.DataExchange().ok(), "Battery unary telemetry failed");
        {
            std::lock_guard<std::mutex> lock(mock.observed_mutex);
            require(mock.observed_battery &&
                    mock.observed_battery_percentage > 0.624f &&
                    mock.observed_battery_percentage < 0.626f,
                    "Battery percentage was lost in existing Sensors transport");
        }

        // A robot may stop reporting BatteryState (or have no battery at all).
        // The next complete unary snapshot must clear its presence while
        // retaining the independent v2 identity and capability descriptor.
        {
            std::lock_guard<std::mutex> lock(*client.getMutex());
            sensors->clear_batterystate();
        }
        require(client.DataExchange().ok(), "Battery-less unary reconnect failed");
        {
            std::lock_guard<std::mutex> lock(mock.observed_mutex);
            require(!mock.observed_battery,
                    "Battery-less unary snapshot retained stale BatteryState");
            require(mock.observed_robot_id == "robot-test" &&
                    mock.observed_capability == "manual.velocity" &&
                    mock.observed_axis == "linear.x",
                    "Battery-less unary snapshot lost RobotInfo");
        }

        // Recovering battery telemetry must be reflected in the next full
        // snapshot, without disturbing stable robot identity/capabilities.
        {
            std::lock_guard<std::mutex> lock(*client.getMutex());
            sensors->mutable_batterystate()->set_percentage(0.375f);
            sensors->mutable_batterystate()->set_present(true);
        }
        require(client.DataExchange().ok(), "Restored battery unary exchange failed");
        {
            std::lock_guard<std::mutex> lock(mock.observed_mutex);
            require(mock.observed_battery &&
                        mock.observed_battery_percentage > 0.374f &&
                        mock.observed_battery_percentage < 0.376f,
                    "Restored battery telemetry was not updated");
            require(mock.observed_robot_id == "robot-test" &&
                        mock.observed_capability == "manual.velocity" &&
                        mock.observed_axis == "linear.x",
                    "Restored battery snapshot lost RobotInfo");
        }

        // Zero percent is valid telemetry, not the same as an absent optional
        // BatteryState. Preserve the presence bit across the wire.
        {
            std::lock_guard<std::mutex> lock(*client.getMutex());
            sensors->mutable_batterystate()->set_percentage(0.0f);
            sensors->mutable_batterystate()->set_present(true);
        }
        require(client.DataExchange().ok(), "Zero-percent battery unary exchange failed");
        {
            std::lock_guard<std::mutex> lock(mock.observed_mutex);
            require(mock.observed_battery && mock.observed_battery_percentage == 0.0f,
                    "Zero-percent battery was mistaken for absent BatteryState");
            require(mock.observed_robot_id == "robot-test" &&
                        mock.observed_capability == "manual.velocity" &&
                        mock.observed_axis == "linear.x",
                    "Zero-percent battery snapshot lost RobotInfo");
        }

        // Use a *fresh*, potentially CONNECTING channel to catch the startup
        // race where GetState() gating previously skipped the first snapshot.
        auto stream_channel = grpc::CreateChannel(address, grpc::InsecureChannelCredentials());
        grpcClient streaming_client(stream_channel, controls, sensors, map, false);
        auto rpc = std::async(std::launch::async, [&streaming_client]() {
            return streaming_client.DataStreamExchange();
        });
        for (int i = 0; i < 300 && !mock.received.load(); ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        require(mock.received.load(), "Agent did not transmit first streaming telemetry");

        // The server holds its response for this test. ROS callbacks must still
        // be able to acquire the agent's shared protobuf mutex during the wait.
        auto mutex = streaming_client.getMutex();
        bool available = false;
        for (int i = 0; i < 40; ++i) {
            if (mutex->try_lock()) {
                mutex->unlock();
                available = true;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        mock.release_reply = true;
        require(rpc.wait_for(std::chrono::seconds(3)) == std::future_status::ready,
                "Client stream did not finish after server sent reply");
        require(rpc.get().ok(), "Client stream RPC failed");
        require(available, "ROS mutex blocked by gRPC response wait");
        require(mock.matchesDescriptor(), "Stream lost robot ID or capability/axes");
        {
            std::lock_guard<std::mutex> lock(*mutex);
            require(controls->basecontrol().linearvelocity() == 11.0,
                    "Stream controls were not applied");
        }

        // Clean server EOF must reconnect when retry is enabled. The next
        // stream must receive a fresh RobotInfo snapshot; stop is terminal.
        mock.release_reply.store(true);
        const unsigned clean_before = mock.stream_count.load();
        grpcClient clean_retry(channel, controls, sensors, map, true);
        auto clean_rpc = std::async(std::launch::async, [&clean_retry]() {
            return clean_retry.DataStreamExchange();
        });
        for (int i = 0; i < 1000 &&
             mock.stream_count.load() < clean_before + 2; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        const bool reconnected = mock.stream_count.load() >= clean_before + 2;
        clean_retry.stopStream();
        const bool finished = clean_rpc.wait_for(std::chrono::seconds(3)) ==
                              std::future_status::ready;
        require(finished, "Clean-EOF retry did not stop");
        require(clean_rpc.get().error_code() == grpc::StatusCode::CANCELLED,
                "Clean-EOF retry did not report explicit shutdown");
        require(reconnected, "Clean server EOF did not trigger a fresh stream");
        require(mock.matchesDescriptor(), "Reconnected stream lost RobotInfo");

        // The map transport becomes idle after its first snapshot. Explicit
        // shutdown must cancel that RPC even if map_seq never changes.
        grpcClient idle_map(channel, controls, sensors, map, true);
        auto idle_map_rpc = std::async(std::launch::async, [&idle_map]() {
            return idle_map.MapStream();
        });
        for (int i = 0; i < 300 && mock.map_snapshots.load() == 0; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        const bool map_snapshot_received = mock.map_snapshots.load() == 1;
        const bool remained_idle =
            idle_map_rpc.wait_for(std::chrono::milliseconds(100)) ==
            std::future_status::timeout;
        idle_map.stopStream();
        const bool map_stopped =
            idle_map_rpc.wait_for(std::chrono::seconds(3)) ==
            std::future_status::ready;
        require(map_snapshot_received, "Map stream did not send initial snapshot");
        require(remained_idle, "Unchanged map unexpectedly terminated stream");
        require(map_stopped, "Idle map stream did not stop after cancellation");
        require(idle_map_rpc.get().error_code() == grpc::StatusCode::CANCELLED,
                "Idle map stream did not report explicit cancellation");

        // A clean peer EOF while the map is unchanged must not cause an
        // unchanged map to be retransmitted. Once map_seq advances, the
        // client must detect the closed RPC and reconnect with a fresh snapshot.
        mock.close_next_map.store(true);
        const unsigned map_before = mock.map_snapshots.load();
        grpcClient map_reconnect(channel, controls, sensors, map, true);
        auto reconnect_rpc = std::async(std::launch::async, [&map_reconnect]() {
            return map_reconnect.MapStream();
        });
        for (int i = 0; i < 300 && mock.map_snapshots.load() == map_before; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        const bool first_map_sent = mock.map_snapshots.load() == map_before + 1;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        const bool no_idle_retransmit = mock.map_snapshots.load() == map_before + 1;
        {
            std::lock_guard<std::mutex> lock(*map_reconnect.getMapMutex());
            map->set_map_seq(map->map_seq() + 1);
        }
        for (int i = 0; i < 1000 && mock.map_snapshots.load() < map_before + 2; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        const bool fresh_map_sent = mock.map_snapshots.load() >= map_before + 2;
        const bool correct_map_seq = mock.last_map_seq.load() == map->map_seq();
        map_reconnect.stopStream();
        const bool reconnect_stopped = reconnect_rpc.wait_for(std::chrono::seconds(3)) ==
                                       std::future_status::ready;
        require(first_map_sent, "Clean-EOF map test missed initial snapshot");
        require(no_idle_retransmit, "Unchanged map was retransmitted after EOF");
        require(fresh_map_sent, "Map change did not reconnect after peer EOF");
        require(correct_map_seq, "Reconnected map snapshot had stale sequence");
        require(reconnect_stopped, "Map reconnect did not stop");
        require(reconnect_rpc.get().error_code() == grpc::StatusCode::CANCELLED,
                "Map reconnect did not report explicit shutdown");

        // Pose and zone streams must also be cancellable while idle, without
        // manufacturing changed telemetry to unblock a gRPC operation.
        grpcClient idle_pose(channel, controls, sensors, map, true);
        auto pose_rpc = std::async(std::launch::async, [&idle_pose]() {
            return idle_pose.PoseStream();
        });
        for (int i = 0; i < 300 && mock.pose_snapshots.load() == 0; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        const bool pose_sent = mock.pose_snapshots.load() == 1;
        const bool pose_idle = pose_rpc.wait_for(std::chrono::milliseconds(100)) ==
                               std::future_status::timeout;
        idle_pose.stopStream();
        const bool pose_finished = pose_rpc.wait_for(std::chrono::seconds(3)) ==
                                   std::future_status::ready;
        require(pose_sent, "Pose stream did not send initial snapshot");
        require(pose_idle && mock.pose_snapshots.load() == 1,
                "Unchanged pose was retransmitted");
        require(pose_finished, "Idle pose stream did not stop");
        require(pose_rpc.get().error_code() == grpc::StatusCode::CANCELLED,
                "Idle pose stream did not report cancellation");

        // A clean server EOF must not force retransmission of an unchanged
        // pose. A subsequent pose change must cause the closed RPC to be
        // detected, followed by a fresh snapshot on the replacement stream.
        mock.close_next_pose.store(true);
        const unsigned pose_before = mock.pose_snapshots.load();
        grpcClient pose_reconnect(channel, controls, sensors, map, true);
        auto pose_reconnect_rpc = std::async(std::launch::async, [&pose_reconnect]() {
            return pose_reconnect.PoseStream();
        });
        for (int i = 0; i < 300 && mock.pose_snapshots.load() == pose_before; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        const bool first_pose_sent = mock.pose_snapshots.load() == pose_before + 1;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        const bool pose_not_repeated = mock.pose_snapshots.load() == pose_before + 1;
        {
            std::lock_guard<std::mutex> lock(*pose_reconnect.getMapMutex());
            map->mutable_robotpose()->set_position_x(
                map->robotpose().position_x() + 1.0);
        }
        for (int i = 0; i < 1000 &&
             mock.pose_snapshots.load() < pose_before + 2; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        const bool updated_pose_sent = mock.pose_snapshots.load() >= pose_before + 2;
        pose_reconnect.stopStream();
        const bool pose_reconnect_finished =
            pose_reconnect_rpc.wait_for(std::chrono::seconds(3)) ==
            std::future_status::ready;
        require(first_pose_sent, "Clean-EOF pose test missed initial snapshot");
        require(pose_not_repeated, "Unchanged pose was retransmitted after EOF");
        require(updated_pose_sent, "Pose change did not reconnect after peer EOF");
        require(pose_reconnect_finished, "Pose reconnect did not stop");
        require(pose_reconnect_rpc.get().error_code() == grpc::StatusCode::CANCELLED,
                "Pose reconnect did not report explicit shutdown");

        grpcClient idle_zone(channel, controls, sensors, map, true);
        auto zone_rpc = std::async(std::launch::async, [&idle_zone]() {
            return idle_zone.ZoneStream();
        });
        for (int i = 0; i < 300 && mock.zone_snapshots.load() == 0; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        const bool zone_sent = mock.zone_snapshots.load() == 1;
        const bool zone_idle = zone_rpc.wait_for(std::chrono::milliseconds(100)) ==
                               std::future_status::timeout;
        idle_zone.stopStream();
        const bool zone_finished = zone_rpc.wait_for(std::chrono::seconds(3)) ==
                                   std::future_status::ready;
        require(zone_sent, "Zone stream did not send initial snapshot");
        require(zone_idle && mock.zone_snapshots.load() == 1,
                "Unchanged zone was retransmitted");
        require(zone_finished, "Idle zone stream did not stop");
        require(zone_rpc.get().error_code() == grpc::StatusCode::CANCELLED,
                "Idle zone stream did not report cancellation");

        // The zone stream must recover from a clean server EOF only when
        // zone_seq changes, without resending an unchanged zone snapshot.
        mock.close_next_zone.store(true);
        const unsigned zone_before = mock.zone_snapshots.load();
        grpcClient zone_reconnect(channel, controls, sensors, map, true);
        auto zone_reconnect_rpc = std::async(std::launch::async, [&zone_reconnect]() {
            return zone_reconnect.ZoneStream();
        });
        for (int i = 0; i < 300 && mock.zone_snapshots.load() == zone_before; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        const bool first_zone_sent = mock.zone_snapshots.load() == zone_before + 1;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        const bool zone_not_repeated = mock.zone_snapshots.load() == zone_before + 1;
        {
            std::lock_guard<std::mutex> lock(*zone_reconnect.getMapMutex());
            auto *zone = map->mutable_zone_map()->zones_size() == 0
                ? map->mutable_zone_map()->add_zones()
                : map->mutable_zone_map()->mutable_zones(0);
            zone->set_id(9001);
            map->set_zone_seq(map->zone_seq() + 1);
        }
        for (int i = 0; i < 1000 &&
             mock.zone_snapshots.load() < zone_before + 2; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        const bool updated_zone_sent = mock.zone_snapshots.load() >= zone_before + 2;
        const bool updated_zone_payload = mock.last_zone_id.load() == 9001;
        zone_reconnect.stopStream();
        const bool zone_reconnect_finished =
            zone_reconnect_rpc.wait_for(std::chrono::seconds(3)) ==
            std::future_status::ready;
        require(first_zone_sent, "Clean-EOF zone test missed initial snapshot");
        require(zone_not_repeated, "Unchanged zone was retransmitted after EOF");
        require(updated_zone_sent, "Zone change did not reconnect after peer EOF");
        require(updated_zone_payload, "Reconnected zone snapshot had stale payload");
        require(zone_reconnect_finished, "Zone reconnect did not stop");
        require(zone_reconnect_rpc.get().error_code() == grpc::StatusCode::CANCELLED,
                "Zone reconnect did not report explicit shutdown");

        // A previously populated zone map can become empty. The wire must
        // carry an explicit empty ZoneMap rather than retain the old zone.
        const unsigned empty_zone_before = mock.zone_snapshots.load();
        {
            std::lock_guard<std::mutex> lock(*zone_reconnect.getMapMutex());
            map->mutable_zone_map()->clear_zones();
            map->set_zone_seq(map->zone_seq() + 1);
        }
        grpcClient empty_zone_client(channel, controls, sensors, map, true);
        auto empty_zone_rpc = std::async(std::launch::async, [&empty_zone_client]() {
            return empty_zone_client.ZoneStream();
        });
        for (int i = 0; i < 300 &&
             mock.zone_snapshots.load() == empty_zone_before; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        const bool empty_zone_received =
            mock.zone_snapshots.load() == empty_zone_before + 1;
        const bool empty_zone_payload =
            mock.last_zone_count.load() == 0 && mock.last_zone_id.load() == 0;
        empty_zone_client.stopStream();
        const bool empty_zone_stopped =
            empty_zone_rpc.wait_for(std::chrono::seconds(3)) ==
            std::future_status::ready;
        require(empty_zone_received, "Empty ZoneMap snapshot not delivered");
        require(empty_zone_payload, "Empty ZoneMap retained stale zones");
        require(empty_zone_stopped, "Empty ZoneMap stream did not stop");
        require(empty_zone_rpc.get().error_code() == grpc::StatusCode::CANCELLED,
                "Empty ZoneMap shutdown did not report cancellation");

        // Verify populated -> empty on one continuously open ZoneStream.
        // A fresh client would only test the initial snapshot path.
        {
            std::lock_guard<std::mutex> lock(*client.getMapMutex());
            auto *zone = map->mutable_zone_map()->add_zones();
            zone->set_id(12345);
            map->set_zone_seq(map->zone_seq() + 1);
        }
        mock.accept_zone_updates.store(true);
        const unsigned live_zone_before = mock.zone_snapshots.load();
        grpcClient live_zone_client(channel, controls, sensors, map, true);
        auto live_zone_rpc = std::async(std::launch::async, [&live_zone_client]() {
            return live_zone_client.ZoneStream();
        });
        for (int i = 0; i < 300 &&
             mock.zone_snapshots.load() < live_zone_before + 1; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        const bool live_populated = mock.zone_snapshots.load() == live_zone_before + 1 &&
                                    mock.last_zone_id.load() == 12345;
        {
            std::lock_guard<std::mutex> lock(*live_zone_client.getMapMutex());
            map->mutable_zone_map()->clear_zones();
            map->set_zone_seq(map->zone_seq() + 1);
        }
        for (int i = 0; i < 300 &&
             mock.zone_snapshots.load() < live_zone_before + 2; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        const bool live_empty = mock.zone_snapshots.load() == live_zone_before + 2 &&
                                mock.last_zone_count.load() == 0 &&
                                mock.last_zone_id.load() == 0;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        const bool no_duplicate_live_zone =
            mock.zone_snapshots.load() == live_zone_before + 2;
        live_zone_client.stopStream();
        const bool live_zone_stopped =
            live_zone_rpc.wait_for(std::chrono::seconds(3)) ==
            std::future_status::ready;
        mock.accept_zone_updates.store(false);
        require(live_populated, "Live ZoneStream missed populated map");
        require(live_empty, "Live ZoneStream did not clear zones on same RPC");
        require(no_duplicate_live_zone, "Live ZoneStream repeated unchanged zones");
        require(live_zone_stopped, "Live ZoneStream did not cancel");
        require(live_zone_rpc.get().error_code() == grpc::StatusCode::CANCELLED,
                "Live ZoneStream cancellation status incorrect");

        // A legacy snapshot must clear metadata observed on the previous
        // v2 unary/stream. The mock must never retain stale capabilities.
        {
            auto client_mutex = client.getMutex();
            std::lock_guard<std::mutex> lock(*client_mutex);
            sensors->clear_robot_info();
        }
        require(client.DataExchange().ok(), "Legacy unary reconnect failed");
        {
            std::lock_guard<std::mutex> lock(mock.observed_mutex);
            require(mock.observed_robot_id.empty() &&
                        mock.observed_capability.empty() &&
                        mock.observed_axis.empty(),
                    "Legacy reconnect retained stale RobotInfo fields");
        }
        // False means one attempt, error propagation and no stale controls.
        mock.fail_next_unary = true;
        unsigned before = mock.unary_count.load();
        auto start = std::chrono::steady_clock::now();
        const grpc::Status failed = client.DataExchange();
        require(failed.error_code() == grpc::StatusCode::UNAVAILABLE,
                "One-shot unary did not propagate disconnect");
        require(mock.unary_count.load() == before + 1,
                "One-shot unary retried unexpectedly");
        require(std::chrono::steady_clock::now() - start < std::chrono::seconds(3),
                "One-shot unary did not fail promptly");
        {
            std::lock_guard<std::mutex> lock(*client.getMutex());
            require(controls->basecontrol().linearvelocity() == 11.0,
                    "Failed unary overwrote a valid control reply");
        }

        // One transient failure must recover when retry is enabled,
        // without blocking ROS callback access during the backoff.
        grpcClient retrying(channel, controls, sensors, map, true);
        mock.fail_next_unary = true;
        before = mock.unary_count.load();
        auto pending = std::async(std::launch::async, [&retrying]() {
            return retrying.DataExchange();
        });
        for (int i = 0; i < 100 && mock.unary_count.load() == before; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        require(mock.unary_count.load() == before + 1,
                "Retrying client did not start");
        bool unlocked = false;
        for (int i = 0; i < 40; ++i) {
            if (retrying.getMutex()->try_lock()) {
                retrying.getMutex()->unlock();
                unlocked = true;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        require(unlocked, "Retry backoff held ROS mutex");
        require(pending.wait_for(std::chrono::seconds(5)) == std::future_status::ready,
                "Retry after transient disconnect timed out");
        require(pending.get().ok(), "Retry-enabled unary did not recover");
        require(mock.unary_count.load() == before + 2,
                "Retry-enabled unary performed unexpected extra attempts");

        // A blocked unary RPC must be canceled by stopStream as well; no
        // stale controls may be applied, and a stopped client is terminal.
        mock.hold_unary.store(true);
        mock.unary_waiting.store(false);
        grpcClient unary_cancel(channel, controls, sensors, map, true);
        auto unary_pending = std::async(std::launch::async, [&unary_cancel]() {
            return unary_cancel.DataExchange();
        });
        for (int i = 0; i < 300 && !mock.unary_waiting.load(); ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        const bool unary_started = mock.unary_waiting.load();
        unary_cancel.stopStream();
        const bool unary_finished =
            unary_pending.wait_for(std::chrono::seconds(3)) == std::future_status::ready;
        mock.hold_unary.store(false);
        require(unary_started, "Unary cancellation did not reach mock");
        require(unary_finished, "stopStream did not cancel unary RPC");
        require(unary_pending.get().error_code() == grpc::StatusCode::CANCELLED,
                "Stopped unary RPC returned unexpected status");
        require(unary_cancel.DataExchange().error_code() == grpc::StatusCode::CANCELLED,
                "Stopped client restarted unary RPC");
        {
            std::lock_guard<std::mutex> lock(*unary_cancel.getMutex());
            require(controls->basecontrol().linearvelocity() == 11.0,
                    "Canceled unary RPC changed previously valid controls");
        }

        // Simulate a server that holds the reply. stopStream must cancel
        // the outstanding gRPC Read and preserve the last controls snapshot.
        mock.received.store(false);
        mock.release_reply.store(false);
        grpcClient cancellable(channel, controls, sensors, map, true);
        auto blocked = std::async(std::launch::async, [&cancellable]() {
            return cancellable.DataStreamExchange();
        });
        for (int i = 0; i < 300 && !mock.received.load(); ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        require(mock.received.load(), "Cancel test did not reach the mock");
        require(blocked.wait_for(std::chrono::milliseconds(50)) ==
                    std::future_status::timeout,
                "Mock did not block the stream response");
        const auto stop_start = std::chrono::steady_clock::now();
        cancellable.stopStream();
        require(blocked.wait_for(std::chrono::seconds(3)) ==
                    std::future_status::ready, "stopStream did not unblock Read");
        const grpc::Status stopped = blocked.get();
        require(stopped.error_code() == grpc::StatusCode::CANCELLED,
                "stopStream did not return CANCELLED");
        require(std::chrono::steady_clock::now() - stop_start <
                    std::chrono::seconds(3), "stopStream took too long");
        {
            std::lock_guard<std::mutex> lock(*cancellable.getMutex());
            require(controls->basecontrol().linearvelocity() == 11.0,
                    "Cancellation overwrote previously valid controls");
        }
        require(cancellable.DataStreamExchange().error_code() ==
                    grpc::StatusCode::CANCELLED,
                "Stopped client restarted a stream");

        // stopStream must cancel every in-flight context, not only the most
        // recently registered stream. Two calls share one client instance.
        mock.release_reply.store(false);
        const unsigned streams_before = mock.stream_count.load();
        grpcClient multi_stream(channel, controls, sensors, map, true);
        auto first = std::async(std::launch::async, [&multi_stream]() {
            return multi_stream.DataStreamExchange();
        });
        auto second = std::async(std::launch::async, [&multi_stream]() {
            return multi_stream.DataStreamExchange();
        });
        for (int i = 0; i < 300 &&
             mock.stream_count.load() < streams_before + 2; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        const bool both_started = mock.stream_count.load() == streams_before + 2;
        multi_stream.stopStream();
        multi_stream.stopStream(); // repeated shutdown is safe
        const bool both_finished =
            first.wait_for(std::chrono::seconds(3)) == std::future_status::ready &&
            second.wait_for(std::chrono::seconds(3)) == std::future_status::ready;
        require(both_started, "Concurrent streams did not reach operator");
        require(both_finished, "stopStream did not cancel both active contexts");
        require(first.get().error_code() == grpc::StatusCode::CANCELLED &&
                    second.get().error_code() == grpc::StatusCode::CANCELLED,
                "Concurrent stream cancellation returned wrong status");
        {
            std::lock_guard<std::mutex> lock(*multi_stream.getMutex());
            require(controls->basecontrol().linearvelocity() == 11.0,
                    "Concurrent cancellation changed valid controls");
        }

        // A single client may have a blocked unary and a blocked stream at
        // the same time. Shutdown must cancel both contexts, preserve the
        // last valid controls, and never retry either RPC afterwards.
        mock.hold_unary.store(true);
        mock.unary_waiting.store(false);
        mock.release_reply.store(false);
        const unsigned mixed_stream_before = mock.stream_count.load();
        grpcClient mixed(channel, controls, sensors, map, true);
        auto mixed_unary = std::async(std::launch::async, [&mixed]() {
            return mixed.DataExchange();
        });
        auto mixed_stream = std::async(std::launch::async, [&mixed]() {
            return mixed.DataStreamExchange();
        });
        for (int i = 0; i < 300 &&
             (!mock.unary_waiting.load() ||
              mock.stream_count.load() < mixed_stream_before + 1); ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        const bool mixed_both_started = mock.unary_waiting.load() &&
            mock.stream_count.load() >= mixed_stream_before + 1;
        mixed.stopStream();
        const bool mixed_both_finished =
            mixed_unary.wait_for(std::chrono::seconds(3)) == std::future_status::ready &&
            mixed_stream.wait_for(std::chrono::seconds(3)) == std::future_status::ready;
        mock.hold_unary.store(false);
        require(mixed_both_started, "Mixed unary and stream did not reach operator");
        require(mixed_both_finished, "Mixed unary and stream were not both canceled");
        require(mixed_unary.get().error_code() == grpc::StatusCode::CANCELLED &&
                    mixed_stream.get().error_code() == grpc::StatusCode::CANCELLED,
                "Mixed cancellation returned unexpected status");
        require(mixed.DataExchange().error_code() == grpc::StatusCode::CANCELLED &&
                    mixed.DataStreamExchange().error_code() == grpc::StatusCode::CANCELLED,
                "Mixed stopped client restarted an RPC");
        {
            std::lock_guard<std::mutex> lock(*mixed.getMutex());
            require(controls->basecontrol().linearvelocity() == 11.0,
                    "Mixed cancellation overwrote valid controls");
        }

        // Stopping during a retry backoff must report CANCELLED, not the
        // transient UNAVAILABLE that triggered the retry. The mock's
        // fail-next flag is consumed only after the first RPC reaches it.
        mock.fail_next_stream.store(true);
        grpcClient backoff_stream(channel, controls, sensors, map, true);
        auto backoff_rpc = std::async(std::launch::async, [&backoff_stream]() {
            return backoff_stream.DataStreamExchange();
        });
        for (int i = 0; i < 300 && mock.fail_next_stream.load(); ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        const bool backoff_failure_delivered = !mock.fail_next_stream.load();
        // Let the first failed RPC return and enter the retry delay.
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        backoff_stream.stopStream();
        const bool backoff_finished =
            backoff_rpc.wait_for(std::chrono::seconds(3)) == std::future_status::ready;
        require(backoff_failure_delivered, "Retry backoff test did not reach operator");
        require(backoff_finished, "Stop during stream retry backoff did not finish");
        require(backoff_rpc.get().error_code() == grpc::StatusCode::CANCELLED,
                "Stopped stream backoff leaked transient failure status");
        {
            std::lock_guard<std::mutex> lock(*backoff_stream.getMutex());
            require(controls->basecontrol().linearvelocity() == 11.0,
                    "Stream retry shutdown changed valid controls");
        }

        // The streaming transport must obey the same one-shot policy.
        mock.fail_next_stream = true;
        grpcClient one_shot_stream(channel, controls, sensors, map, false);
        start = std::chrono::steady_clock::now();
        const grpc::Status stream_failed = one_shot_stream.DataStreamExchange();
        require(stream_failed.error_code() == grpc::StatusCode::UNAVAILABLE,
                "One-shot stream did not surface peer failure");
        require(std::chrono::steady_clock::now() - start < std::chrono::seconds(3),
                "One-shot stream retried or blocked unexpectedly");
        {
            std::lock_guard<std::mutex> lock(*one_shot_stream.getMutex());
            require(controls->basecontrol().linearvelocity() == 11.0,
                    "Failed stream overwrote previously valid controls");
        }
    } catch (const std::exception &e) {
        mock.release_reply = true;
        server->Shutdown();
        std::cerr << "Agent RPC smoke failed: " << e.what() << std::endl;
        return 1;
    }
    server->Shutdown();
    std::cout << "rc_agent unary/stream and idle mutex smoke: PASS" << std::endl;
    return 0;
}
