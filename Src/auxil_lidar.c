#include  "main.h"
#include <string.h>
#include <stdlib.h>

lidar_data lidar;

uint8_t LiDAR_CalculateChecksum(const uint8_t *frame)
{
    uint16_t sum = 0; // Use uint16_t to prevent overflow before masking

    // Sum bytes 0 through 7
    for (uint8_t i = 0; i < 8; i++)
        sum += frame[i];
    // Return the lower 8 bits (equivalent to sum % 256)
    return (uint8_t)(sum & 0xFF);
}

/**
 * @brief Validates the LiDAR frame.
 * @param frame Pointer to the array of 9 bytes.
 * @return true if valid, false if checksum fails or header is wrong.
 */
uint8_t LiDAR_ValidateFrame(const uint8_t *frame)
{
    // 1. Check frame headers (Bytes 0 and 1 must be 0x59)
    if (frame[0] != 0x59 || frame[1] != 0x59)
        return 0;

    // 2. Calculate and compare checksum with Byte 8
    uint8_t calculated_checksum = LiDAR_CalculateChecksum(frame);
    if (calculated_checksum != frame[8])
        return 0; // Checksum mismatch
    return 1; // Frame is valid!
}


// Assuming you have filled rx_buffer with 9 bytes from UART:
void ProcessLidar(const uint8_t *frame)
{
    lidar.valid=0;
	if (LiDAR_ValidateFrame(frame))
	{
        // Extract Distance (Bytes 2 & 3)
        lidar.distance = (uint16_t)frame[2] | ((uint16_t)frame[3] << 8);

        // Extract Strength (Bytes 4 & 5)
        lidar.strength = (uint16_t)frame[4] | ((uint16_t)frame[5] << 8);

        // TODO: Do something with distance and strength (e.g., printf or update variables)
        lidar.valid=1;
    }
}
