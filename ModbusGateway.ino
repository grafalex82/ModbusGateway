#include <ESP8266WiFi.h>
#include "WiFiCredentials.h"

#define DebugSerial Serial1

size_t connectionAttempts = 100;

// Modbus constants
size_t MBAPSize = 7; // Size of MBAP block (Modbus TCP message header)

// An instance of the ModBus TCP server
WiFiServer server(502);

// RS485 is half duplex and requires manual switch between transfer directions
uint8_t dirPin = 12;

enum DebugLevel
{
    Normal,
    Verbose,
    Debug
};

DebugLevel debugLevel = Verbose;

template<class T>
void debugPrint(DebugLevel level, T msg)
{
    if(debugLevel >= level)
        DebugSerial.print(msg);
}

template<class T>
void debugPrintLn(DebugLevel level, T msg)
{
    if(debugLevel >= level)
        DebugSerial.println(msg);
}

uint16_t bytesToU16(const uint8_t * buf)
{
    return (buf[0] << 8) + buf[1];
}

// Calculate CRC16 is required by Modbus RTU
uint16_t crc16(const uint8_t * buf, size_t len)
{
    uint16_t crc = 0xffff;
    
    while(len-- > 0)
    {
        crc ^= (uint16_t)(*buf++);          // XOR byte into least sig. byte of crc

        for (int i = 0; i < 8; i++)         // Loop over each bit
        {
            if ((crc & 0x0001) != 0)        // If the LSB is set
            {
                crc >>= 1;                  // Shift right and XOR 0xA001
                crc ^= 0xA001;
            }
            else                            // Else LSB is not set
            {
                crc >>= 1;                  // Just shift right
            }
        }
    }
    
    return crc;
}

void dumpBuf(const char * label, const uint8_t * buf, size_t len)
{
    // No need to spam the console unless we are debugging
    if(debugLevel < Debug)
      return;

    // Print the packet
    DebugSerial.print(label);
    DebugSerial.print(": ");
    for(size_t i=0; i<len; i++)
    {
        DebugSerial.print(buf[i], HEX);
        DebugSerial.print(' ');
    }
    DebugSerial.println();
}

void sendRS485Packet(const uint8_t * buf, size_t len)
{
    //Switch transciever to transmit mode
    digitalWrite(dirPin, HIGH);

    dumpBuf("Sending packet", buf, len);

    // Send the data
    Serial.write(buf, len);
    Serial.flush(); // Always wait until data is really sent

    // Wait for one more byte to go
    uint16_t us = 1000000 / (9600 / 10);
    delayMicroseconds(us);
}

size_t receiveRS485Response(uint8_t * buf)
{
    // Flush rxbuf before starting receiving response
    Serial.readBytes(buf, Serial.available());
    
    //Switch transciever to receive mode
    digitalWrite(dirPin, LOW);

    // Receive the response header
    size_t rcvd = Serial.readBytes(buf, 3); // Receive the header - 3rd byte contains lengh of the payload
    if(rcvd != 3)
    {
        debugPrintLn(Normal, "Timeout");
        debugPrint(Verbose, "Invalid RS485 response header length ");
        debugPrintLn(Verbose, rcvd);

        // Fill the exception code
        // buf[0] = ... // Hopefully function code byte is still there
        buf[1] |= 0x80; // Set the exception code
        buf[2] = 0x0b;
        
        return 3;
    }

    // Receive remaining of the data payload
    uint8_t payloadLen = buf[2]; //3rd byte contains lengh of the payload
    rcvd += Serial.readBytes(buf + 3, payloadLen + 2); // Read 2 additional bytes to account CRC
    rcvd -= 2; // Do not return CRC

    // TODO: Do we have to check CRC?

    return rcvd;
}

// Send request and receive response over modbus
// Function adds CRC16 automatically
// Buffer shall be big enough to fit CRC as well as response
size_t processModbusRTURequest(uint8_t * buf, size_t len)
{
    // Modbus RTU requires a delay between requests for at least 3.5 characters, which is ~4ms at 9600
    // I'll use a bigger delay, just in case
    delay(100);
  
    // Add CRC first
    uint16_t crc = crc16(buf, len);
    buf[len] = crc & 0xff;
    buf[len+1] = crc >> 8;
    len += 2; // to account added bytes

    dumpBuf("Request", buf, len);

    // Send the packet
    sendRS485Packet(buf, len);

    // Wait for response
    size_t responseLen = receiveRS485Response(buf);

    dumpBuf("Response", buf, responseLen);

    return responseLen;
}

// Process Modbus TCP formatted request over Modbus RTU
// Buffer shall be big enough to fit CRC as well as response
// Response will be stored in the same buffer
size_t processModbusTCPRequest(uint8_t * buf)
{
    // Header (MBAP) format:
    // 2 bytes - Transaction ID - leave as is
    // 2 bytes - Protocol ID (always 0)
    // 2 byte - size
    // rest of the data - packet, same format as Modbus RTU

    uint16_t len = buf[4];
    size_t responseLen = processModbusRTURequest(buf + 6, len);

    return responseLen;
}

bool connectToWiFi()
{
    // Check if we connected already
    if(WiFi.status() == WL_CONNECTED)
        return true;
    
    // Connect to WiFi network
    debugPrintLn(Normal, "");
    debugPrintLn(Normal, "");
    debugPrint(Normal, "Connecting to ");
    debugPrint(Normal, ssid);
  
    WiFi.begin(ssid, password);
    for(size_t i=0; i<connectionAttempts; i++)
    {
        delay(100);
        if(WiFi.status() == WL_CONNECTED)
        {
            debugPrintLn(Normal, "");
            debugPrintLn(Normal, "WiFi connected");
            debugPrintLn(Normal, WiFi.localIP());

            return true;
        }

        debugPrint(Normal, ".");
    }

    return false;
}

void setup()
{
    // Initialize transfer direction pin
    pinMode(dirPin, OUTPUT);

    // Initialize debug serial    
    DebugSerial.begin(115200);
    delay(100);

    // Initialize RS485 serial on alternalte pins
    Serial.begin(9600);
    Serial.swap();


#if 0
    // A code for basic set up of a counter 
    delay(2000);
    uint8_t fbuf[4];

    // Set up baud rate (set baudrate to 2400 a few lines above)
    //uint8_t buf[100] = {/* default ID */0x01, /* Set holding register */0x10, /* register address*/ 0x00, 0x1c , /* number of registers (2 bytes) */0x00, 0x02, /*number of bytes*/0x04};
    //*(float *)(fbuf) = 2.; // 9600 baud rate

    // Set up device ID
    uint8_t buf[100] = {/* default ID */0x01, /* Set holding register */0x10, /* register address*/ 0x00, 0x14 , /* number of registers (2 bytes) */0x00, 0x02, /*number of bytes*/0x04};
    *(float *)(fbuf) = 6.; //An ID
       
    buf[7] = fbuf[3];
    buf[8] = fbuf[2];
    buf[9] = fbuf[1];
    buf[10] = fbuf[0];
    
    debugPrintLn(Normal, "Setting up counter ID");
    size_t responseSize = processModbusRTURequest(buf, 11);
    dumpBuf("Response", buf, responseSize);

    while(1);
#endif
}

bool receiveTCPData(WiFiClient & client, uint8_t * buf, size_t len)
{
    uint16_t elapsed = 0;
    while((size_t)client.available() < len)
    {
        if(!client.connected())
        {
          debugPrintLn(Verbose, "receiveTCPData - not connected");
          return false;
        }
      
        elapsed++;
        //if(elapsed >= 10000)
        //    return false;
            
        // TODO: add error processing here
        delay(1);
    }

    client.read(buf, len);
    return true;
}

bool processTCPRequest(WiFiClient & client)
{
    uint8_t buf[300];

    // Waiting for MBAP (Message header)
    debugPrintLn(Debug, "Waiting for MBAP");
    if(!receiveTCPData(client, buf, MBAPSize))
    {
        debugPrintLn(Debug, "Waiting MBAP failed");
        return false;
    }

    dumpBuf("MBAP", buf, MBAPSize);
    uint16_t packetSize = bytesToU16(buf + 4) - 1; // Size includes the last byte of MBAP - do not count it

    debugPrint(Debug, "Packet size:");
    debugPrintLn(Debug, packetSize);
    
    if(!receiveTCPData(client, buf + MBAPSize, packetSize))
        return false;

    debugPrint(Verbose, "Request: transaction ");
    debugPrint(Verbose, bytesToU16(buf));
    debugPrint(Verbose, " unit ");
    debugPrint(Verbose, buf[6]);
    debugPrint(Verbose, " function ");
    debugPrint(Verbose, buf[7]);
    debugPrint(Verbose, " register ");
    debugPrint(Verbose, bytesToU16(buf + 8));
    debugPrint(Verbose, " count ");
    debugPrint(Verbose, bytesToU16(buf + 10));
    debugPrint(Verbose, ": ");


    dumpBuf("Pscket", buf, MBAPSize + packetSize);

    debugPrintLn(Debug, "Processing the request");
    size_t responseSize = processModbusRTURequest(buf + MBAPSize - 1, packetSize + 1);

    // Store response size in MBAP
    buf[4] = (responseSize >> 8) & 0xff;
    buf[5] = responseSize & 0xff;

    dumpBuf("Response", buf, responseSize + MBAPSize - 1);
    client.write((const uint8_t *)buf, responseSize + MBAPSize - 1);

    if(responseSize > 3)
      debugPrintLn(Verbose, "OK");
    debugPrintLn(Debug, "Request processed");
    debugPrintLn(Debug, "");

    return true;
}

void loop()
{
    delay(1000);

    // Connect to WiFi first
    if(!connectToWiFi())
        return;

    //DebugSerial.println("Start a new cycle");
    while(true)
    {
        //DebugSerial.println("server::begin()");
        server.begin();
        
        // Check if a client has connected
        WiFiClient client = server.available();
        if (!client)
        {
            //DebugSerial.println("no client...");
            break;
        }
            
        // Listen for packets
        while(processTCPRequest(client))
        {
            //DebugSerial.println("Packet processed fine. Going to process another request on the same session");
        }
    }
}
