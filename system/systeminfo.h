#ifndef SYSTEMINFO_H
#define SYSTEMINFO_H

#include <string>
#include "shell.h"

namespace System {
namespace Info {

    /**
    * @brief Retrieves the systemd machine ID.
    *
    * Executes the shell command to read the machine ID from "/etc/machine-id".
    *
    * @return std::string The machine ID as a string.
    */
    std::string getMachineID();

    /**
    * @brief Retrieves the system UUID.
    *
    * Executes the shell command to read the product UUID from "/sys/class/dmi/id/product_uuid".
    *
    * @return std::string The system UUID as a string.
    */
    std::string getUUID();

    /**
    * @brief Retrieves the product serial number.
    *
    * Executes the shell command to read the product serial from "/sys/class/dmi/id/product_serial".
    *
    * @return std::string The product serial number as a string.
    */
    std::string getSerialProduct();

    /**
    * @brief Retrieves the chassis serial number.
    *
    * Executes the shell command to read the chassis serial from "/sys/class/dmi/id/chassis_serial".
    *
    * @return std::string The chassis serial number as a string.
    */
    std::string getSerialChassis();

    /**
    * @brief Retrieves the board serial number.
    *
    * Executes the shell command to read the board serial from "/sys/class/dmi/id/board_serial".
    *
    * @return std::string The board serial number as a string.
    */
    std::string getSerialBoard();

} // namespace Info
} // namespace System
#endif // SYSTEMINFO_H
