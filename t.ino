/***********************************************************************************************************************
* @FILE serial_communication_variable.ino
* @BRIEF An example Arduino sketch showing USB-serial communications with the Teensy microcontroller
*
* This program provides an example of USB-serial communications with the Teensy 3.1/3.2 microcontroller. The communication
* is based on variable width byte packets containing an error checksum. The packet structure is defined as follows:
*
* packet[0] = PACKET_START_BYTE (0xAA)
* packet[1] = PACKET_SIZE (total number of bytes including overhead and payload)
* packet[n+2] = payload byte n -> [0, PAYLOAD_SIZE - 1]
* packet[PACKET_SIZE - 1] = packet checksum
*
* The checksum is computed as the XOR chain of each byte in the packet before the checksum:
* packet[0] XOR packet[1] XOR ... XOR packet[PACKET_SIZE - 2]
*
* @AUTHOR Christopher D. McMurrough
**********************************************************************************************************************/

// define GPIO pins
const int LED_PIN = 13;
const int LED_ON = HIGH;
const int LED_OFF = LOW;

// define serial communication parameters
const unsigned long BAUD_RATE = 9600;

// define packet parameters
const byte PACKET_START_BYTE = 0xAA;
const unsigned int PACKET_OVERHEAD_BYTES = 4;
const unsigned int PACKET_MIN_BYTES = PACKET_OVERHEAD_BYTES + 1;
const unsigned int PACKET_MAX_BYTES = 255;

/***********************************************************************************************************************
* @BRIEF perform initial setup of the microcontroller
* @AUTHOR Christoper D. McMurrough
**********************************************************************************************************************/
void setup()
{
    // initialize the IO pins
    pinMode(LED_PIN, OUTPUT);

    // initialize the serial port
    Serial.begin(BAUD_RATE);

    // flash the LED state
    for(int i = 0; i < 25; i++)
    {
        digitalWrite(LED_PIN, LED_ON);
        delay(50);
        digitalWrite(LED_PIN, LED_OFF);
        delay(50);
    }
}

/***********************************************************************************************************************
* @BRIEF assembles and transmits a serial packet containing the given payload
* @PARAM[in] payloadSize the size of the given payload in bytes
* @PARAM[in] payload pointer to the data payload array
* @RETURN true if the packet was transmitted successfully
* @AUTHOR Christoper D. McMurrough
**********************************************************************************************************************/
boolean sendPacket(unsigned int payloadSize, byte *payload)
{
    // check for max payload size
    unsigned int packetSize = payloadSize + PACKET_OVERHEAD_BYTES;
    if(packetSize > PACKET_MAX_BYTES)
    {
        return false;
    }

    // create the serial packet transmit buffer
    static byte packet[PACKET_MAX_BYTES];

    // populate the overhead fields
    packet[0] = PACKET_START_BYTE;
    packet[1] = packetSize;
    byte checksum[2];
    checksum[0] = packet[0] ^ packet[2];
    checksum[1] = packet[1] ^ packet[3];

    // populate the packet payload while computing the checksum
    for(int i = 0; i < payloadSize; i++)
    {
        packet[i + 2] = payload[i];
        checkSum[i%2] = checkSum[i%2] ^ packet[i + 4];
    }

    // store the checksum
    packet[packetSize - 2] = checksum[0];
    packet[packetSize - 1] = checksum[1];

    // send the packet
    Serial.write(packet, packetSize);
    Serial.flush();
    return true;
}


/***********************************************************************************************************************
* @BRIEF checks to see if the given packet is complete and valid
* @PARAM[in] packetSize the size of the given packet buffer in bytes
* @PARAM[in] packet pointer to the packet buffer
* @RETURN true if the packet is valid
* @AUTHOR Christoper D. McMurrough
**********************************************************************************************************************/
boolean validatePacket(unsigned int packetSize, byte *packet)
{
    // check the packet size
    if(packetSize < PACKET_MIN_BYTES || packetSize > PACKET_MAX_BYTES)
    {
        return false;
    }

    // check the start byte
    if(packet[0] != PACKET_START_BYTE)
    {
        return false;
    }

    // check the length byte
    if(packet[1] != packetSize)
    {
        return false;
    }

    // compute the checksum
    byte checksum[2];
    checksum[0] = 0x00;
    checksum[1] = 0x00;

    for(int i = 0; i < packetSize - 2; i++)
    {
        checksum[j%2] = checksum[j%2] ^ packet[j];
    }

    // check to see if the computed checksum and packet checksum are equal
    if((packet[packetSize - 2] != checksum[0]) || (packet[packetSize - 1] != checksum[1])))
    {
        return false;
    }

    // all validation checks passed, the packet is valid
    return true;
}

/***********************************************************************************************************************
* @BRIEF main program loop
* @AUTHOR Christoper D. McMurrough
**********************************************************************************************************************/
void loop()
{
    // define control variables
    boolean isRunning = true;
    boolean ledState = false;

    // create the serial packet receive buffer
    static byte buffer[PACKET_MAX_BYTES];
    int count = 0;
    int packetSize = PACKET_MIN_BYTES;

    // continuously check for received packets
    while(isRunning)
    {
        // check to see if serial byte is available
        if(Serial.available())
        {
            // get the byte
            byte b = Serial.read();

            // handle the byte according to the current count
            if(count == 0 && b == PACKET_START_BYTE)
            {
                // this byte signals the beginning of a new packet
                buffer[count] = b;
                count++;
                continue;
            }
            else if(count == 0)
            {
                // the first byte is not valid, ignore it and continue
                continue;
            }
            else if(count == 1)
            {
                // this byte contains the overall packet length
                buffer[count] = b;

                // reset the count if the packet length is not in range
                if(packetSize < PACKET_MIN_BYTES || packetSize > PACKET_MAX_BYTES)
                {
                    count = 0;
                }
                else
                {
                    packetSize = b;
                    count++;
                }
                continue;
            }
            else if(count < packetSize)
            {
                // store the byte
                buffer[count] = b;
                count++;
            }

            // check to see if we have acquired enough bytes for a full packet
            if(count >= packetSize)
            {
                // validate the packet
                if(validatePacket(packetSize, buffer))
                {
                    // change the LED state if the packet is valid
                    ledState = !ledState;
                    digitalWrite(LED_PIN, ledState);

                    // echo back the received packet payload
                    sendPacket(packetSize - PACKET_OVERHEAD_BYTES, buffer + 2);
                }
                // reset the count
                count = 0;
            }
        }
    }
}
