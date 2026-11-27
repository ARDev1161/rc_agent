#pragma once

#include <fstream>
#include <iterator>
#include <optional>
#include <stdexcept>
#include <string>
#include <grpcpp/grpcpp.h>
#include "../config/configprocessor.h"

namespace rc::transport {
struct TlsMaterial {
    std::string ca, certificate, private_key;
};

inline std::string readPem(const std::string &path)
{
    std::ifstream input(path, std::ios::binary);
    if (!input) throw std::runtime_error("Cannot read configured TLS PEM: " + path);
    std::string pem((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
    if (input.bad() || pem.empty()) throw std::runtime_error("Empty or unreadable TLS PEM: " + path);
    return pem;
}

// Absent group preserves telemetry-only insecure mode. A present but invalid
// group must fail closed; never silently downgrade configured TLS.
inline std::optional<TlsMaterial> loadTlsMaterial(ConfigProcessor &config)
{
    const std::string prefix = "Amur.Network.tls";
    const auto type = config.getType(prefix);
    if (type == libconfig::Setting::TypeNone) return std::nullopt;
    if (type != libconfig::Setting::TypeGroup)
        throw std::runtime_error("Amur.Network.tls must be a group");
    const auto field = [&](const char *name) {
        std::string path;
        if (config.searchString(prefix + "." + name, path) != EXIT_SUCCESS || path.empty())
            throw std::runtime_error("Missing TLS path: " + std::string(name));
        return readPem(path);
    };
    return TlsMaterial{field("ca_file"), field("certificate_file"), field("private_key_file")};
}

inline std::shared_ptr<grpc::ServerCredentials> makeTlsServerCredentials(const TlsMaterial &tls)
{
    grpc::SslServerCredentialsOptions options;
    options.pem_root_certs = tls.ca;
    options.pem_key_cert_pairs.push_back({tls.private_key, tls.certificate});
    options.client_certificate_request = GRPC_SSL_REQUEST_AND_REQUIRE_CLIENT_CERTIFICATE_AND_VERIFY;
    return grpc::SslServerCredentials(options);
}

inline std::shared_ptr<grpc::ChannelCredentials> makeTlsChannelCredentials(const TlsMaterial &tls)
{
    grpc::SslCredentialsOptions options;
    options.pem_root_certs = tls.ca;
    options.pem_cert_chain = tls.certificate;
    options.pem_private_key = tls.private_key;
    return grpc::SslCredentials(options);
}
} // namespace rc::transport
