#ifndef SYSTEM_H
#define SYSTEM_H

#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <chrono>
#include <sys/times.h>
#include <unistd.h>
#include <vector>

#include "shell.h"

/**
 * @brief Definition of byte type.
 */
typedef uint8_t Byte;

/**
 * @brief Definition of double byte type.
 */
typedef uint16_t doubleByte;

namespace System {
namespace Misc {

/**
 * @brief Creates a temporary file for firmware.
 *
 * This function appends a template "XXXXXX" to the given directory path, generates a unique
 * temporary filename using mkstemp(), opens the file using the provided std::ofstream, and returns
 * the generated path. If an error occurs, an empty string is returned.
 *
 * @param[in] path Directory path for the temporary file.
 * @param[in,out] fileStream Reference to an std::ofstream object that will be opened for writing.
 * @return std::string The path to the temporary file, or an empty string if an error occurred.
 */
std::string makeTemp(std::string path, std::ofstream &fileStream);

/**
 * @brief Converts a vector of bytes to a string.
 *
 * This function concatenates each byte from the input vector into a single output string.
 *
 * @param[in] data Input vector of bytes.
 * @return std::string The resulting string.
 */
std::string vectorToString(const std::vector<Byte> &data);

/**
 * @brief Converts a string to a vector of bytes.
 *
 * This function creates a vector of bytes from the characters of the input string.
 *
 * @param[in] data Input string.
 * @return std::vector<Byte> The resulting vector of bytes.
 */
std::vector<Byte> stringToVector(const std::string &data);

} // namespace Misc
} // namespace System

#endif // SYSTEM_H
