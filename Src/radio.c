#include "main.h"
#define RADIO_HEAD_SIZE 7

#define RADIO_QUEUE_SIZE 8
sradio_data radio_queue[RADIO_QUEUE_SIZE];

uint8_t radio_queue_head = 0;
uint8_t radio_queue_tail = 0;
volatile uint16_t radio_read_pos = 0;
extern UART_HandleTypeDef huart2;
extern uint8_t radio_buff[RADIO_BUFF_SIZE];
extern uint8_t radio_command[RADIO_COMMAND_SIZE];
extern uint16_t radio_old_pos;

void execute_radio_command(const sradio_data *radio_data)
{

}
uint8_t radio_queue_push(const sradio_data *data)
{
    uint8_t next =
        (radio_queue_head + 1) % RADIO_QUEUE_SIZE;

    if (next == radio_queue_tail)
        return 0;       // queue full

    radio_queue[radio_queue_head] = *data;

    radio_queue_head = next;

    return 1;
}

uint8_t radio_queue_pop(sradio_data *data)
{
    if (radio_queue_tail == radio_queue_head)
        return 0;       // empty

    *data = radio_queue[radio_queue_tail];

    radio_queue_tail =
        (radio_queue_tail + 1) % RADIO_QUEUE_SIZE;

    return 1;
}

uint16_t crc16_ccitt(const uint8_t* data, size_t length)
{
    uint16_t crc = 0xFFFF;      // Initial value
    while (length--)
    {
    	crc ^= (uint16_t)(*data++) << 8;
    	for (int i = 0; i < 8; i++)
    	{
    		if (crc & 0x8000)
    			crc = (crc << 1) ^ 0x1021;
    		else
    			crc <<= 1;
    	}
    }
    return crc;
}

uint8_t process_radio(sradio_data *data)
{
    uint16_t dma_pos =
        RADIO_BUFF_SIZE -
        __HAL_DMA_GET_COUNTER(huart2.hdmarx);

    /*
     * Nothing new?
     */
    if (radio_read_pos == dma_pos)
        return -1;


    /*
     * We currently handle packets that do NOT
     * cross the end of the circular buffer.
     *
     * Search from read position to DMA position.
     */

    uint16_t available;

    if (dma_pos >= radio_read_pos)
        available = dma_pos - radio_read_pos;
    else
        available = RADIO_BUFF_SIZE - radio_read_pos;


    if (available < RADIO_HEAD_SIZE)
        return -1;


    for (uint16_t i = radio_read_pos;
         i + RADIO_HEAD_SIZE <=
             radio_read_pos + available;
         i++)
    {
        srhead *tmpr =
            (srhead *)&radio_buff[i];


        /*
         * Header:
         *
         * AA 55 AA 55
         */

        if ((tmpr->head[0] == 0xAA) &&
            (tmpr->head[1] == 0x55) &&
            (tmpr->head[2] == 0xAA) &&
            (tmpr->head[3] == 0x55))
        {
            /*
             * Check payload length
             */

            if (tmpr->length > RADIO_PAYLOAD_SIZE)
            {
                radio_read_pos = i + 1;
                return -2;
            }


            /*
             * Full packet length:
             *
             * header 7
             * payload length
             * CRC 2
             * END 4
             */

            uint16_t packet_size =
                RADIO_HEAD_SIZE +
                tmpr->length +
                sizeof(uint16_t) +
                4;


            /*
             * Do we have the complete packet?
             */

            if ((radio_read_pos + available - i) <
                packet_size)
            {
                return -3;
            }


            /*
             * Copy header
             */

            memcpy(&data->head,
                   &radio_buff[i],
                   RADIO_HEAD_SIZE);


            /*
             * Copy payload
             */

            memcpy(data->payload,
                   &radio_buff[i + RADIO_HEAD_SIZE],
                   data->head.length);


            /*
             * Copy CRC16
             */

            memcpy(&data->chsumm,
                   &radio_buff[i +
                               RADIO_HEAD_SIZE +
                               data->head.length],
                   sizeof(uint16_t));


            /*
             * Copy END
             */

            memcpy(data->end,
                   &radio_buff[i +
                               RADIO_HEAD_SIZE +
                               data->head.length +
                               sizeof(uint16_t)],
                   4);


            /*
             * CRC
             */

            uint16_t crc =
                crc16_ccitt(data->payload,
                            data->head.length);

            if (crc != data->chsumm)
            {
                radio_read_pos = i + 1;
                return -4;
            }


            /*
             * END
             */

            if ((data->end[0] == 0x0D) &&
                (data->end[1] == 0x0A) &&
                (data->end[2] == 0x0D) &&
                (data->end[3] == 0x0A))
            {
                /*
                 * Packet successfully consumed.
                 */

                radio_read_pos =
                    i + packet_size;

                if (radio_read_pos >= RADIO_BUFF_SIZE)
                    radio_read_pos -= RADIO_BUFF_SIZE;

                return 0;
            }

            /*
             * Bad END.
             */

            radio_read_pos = i + 1;

            if (radio_read_pos >= RADIO_BUFF_SIZE)
                radio_read_pos = 0;

            return -5;
        }
    }


    /*
     * No header found.
     *
     * We can discard the bytes we searched.
     */

    radio_read_pos = dma_pos;

    return -6;
}


uint8_t process_radio1(sradio_data *data)
{
	uint16_t pos = RADIO_BUFF_SIZE - __HAL_DMA_GET_COUNTER(huart2.hdmarx);
	if(pos<7)
		return -1;
//	for(uint16_t i=0;i<pos;i++)
	for (uint16_t i = 0; i + RADIO_HEAD_SIZE <= pos; i++)
	{
		srhead* tmpr=(srhead*)&radio_buff[i];
		if(
			(tmpr->head[0]==tmpr->head[2])&&
			(tmpr->head[1]==tmpr->head[3])&&
			(tmpr->head[0]==0xaa)&&
			(tmpr->head[1]==0x55)
		)
		{
			if(tmpr->length>RADIO_PAYLOAD_SIZE)
				return -2;
			if((pos-i)<tmpr->length+13)//not full data present;
				return -3;

			//all data available so copy data to sradio_data;
			memcpy(&data->head,radio_buff+i,RADIO_HEAD_SIZE);//copy header
			memcpy(data->payload,radio_buff+i+RADIO_HEAD_SIZE,data->head.length);//copy payload
			memcpy(&data->chsumm,radio_buff+i+RADIO_HEAD_SIZE+data->head.length,sizeof(uint16_t));
			memcpy(data->end,radio_buff+i+RADIO_HEAD_SIZE+data->head.length+sizeof(uint16_t),4);
			uint16_t crc=crc16_ccitt(data->payload,data->head.length);
			if(crc!=data->chsumm)
				return -4;
			if((data->end[0]==data->end[2])&&
				(data->end[1]==data->end[3])&&
				(data->end[0]==0x0d)&&(data->end[1]==0x0a))
				return 0;//packet OK
		}

	}
	return -5;
}

#define COMMAND_QUEUE_SIZE 20  // Maximum number of pending commands in queue

// Structure for a valid command packet
static RadioPacket command_queue[COMMAND_QUEUE_SIZE];
static uint8_t queue_head = 0;
static uint8_t queue_tail = 0;
static uint8_t queue_count = 0;
void exec_radio(RadioPacket *pkt)
{
}
// Push parsed packet into the static queue
uint8_t r_push_to_queue(uint8_t cmd, uint16_t length, const uint8_t *payload)
{
    if (queue_count >= COMMAND_QUEUE_SIZE)
    {
        return 0; // Queue is full (Overflow protection)
    }

    command_queue[queue_tail].cmd = cmd;
    command_queue[queue_tail].length = length;

    if (length > 0 && length < RADIO_PAYLOAD_SIZE)
    {
        memcpy(command_queue[queue_tail].payload, payload, length);
    }

    queue_tail = (queue_tail + 1) % COMMAND_QUEUE_SIZE;
    queue_count++;
    return 1;
}

// Pull packet from the queue (used in your main loop)
uint8_t r_pop_from_queue(RadioPacket *pkt) \
{
    if (queue_count == 0)
    {
        return 0; // Queue is empty
    }

    *pkt = command_queue[queue_head];
    queue_head = (queue_head + 1) % COMMAND_QUEUE_SIZE;
    queue_count--;
    return 1;
}

uint8_t process_radio_stream()
{
    size_t index = 0;
    uint8_t parsed_count = 0;

    const uint8_t start_marker[4] = {0xAA, 0x55, 0xAA, 0x55};
    const uint8_t end_marker[4] = {0x0D, 0x0A, 0x0D, 0x0A};

    // Minimum packet size: Start(4) + Cmd(1) + Length(2) + Payload(min 0) + CRC(2) + End(4) = 13 bytes
    while (index + 13 <= radio_line_size) {

        // 1. Look for Start Marker
        if (memcmp(radio_command + index, start_marker, 4) != 0) {
            index++; // Shift by 1 byte to search for sync bytes
            continue;
        }

        // 2. Read Command ID and Length
        uint8_t cmd = radio_command[index + 4];
        uint16_t length;
        memcpy(&length, radio_command + index + 5, sizeof(uint16_t));

        // Sanity check: Ensure payload length is within bounds (< 256)
        if (length >= RADIO_PAYLOAD_SIZE) {
            index += 4; // Corrupted length field, skip past start marker to recover
            continue;
        }

        // Calculate total packet size: Head(4) + Cmd(1) + Length(2) + Payload(N) + CRC(2) + Tail(4)
        size_t total_packet_size = 4 + 1 + 2 + length + 2 + 4;

        // 3. Check if the full packet has arrived in the buffer yet
        if (index + total_packet_size > radio_line_size) {
            break; // Incomplete packet at the end; wait for the next UART DMA callback
        }

        // 4. Verify End Marker
        if (memcmp(radio_command + index + total_packet_size - 4, end_marker, 4) != 0) {
            index += 4; // Invalid tail, skip and resync
            continue;
        }

        // 5. Verify Checksum (Calculated over Cmd + Length + Payload)
        size_t crc_data_len = 1 + 2 + length;
        uint16_t received_crc;
        memcpy(&received_crc, radio_command + index + 4 + crc_data_len, sizeof(uint16_t));

        uint16_t calculated_crc = crc16_ccitt(radio_command + index + 4, crc_data_len);

        if (calculated_crc == received_crc) {
            // --- VALID PACKET! Push into static queue ---
            r_push_to_queue(cmd, length, radio_command + index + 7);
            parsed_count++;
        }

        // 6. Advance index past this processed packet
        index += total_packet_size;
    }

    // Shift any remaining unparsed or partial bytes back to the front of radio_command
    if (index > 0) {
        radio_line_size -= index;
        if (radio_line_size > 0) {
            memmove(radio_command, radio_command + index, radio_line_size);
        }
    }

    return parsed_count;
}
