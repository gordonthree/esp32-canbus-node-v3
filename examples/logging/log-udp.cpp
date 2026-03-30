#include <WiFi.h>
#include <lwip/sockets.h>

static int udp_sock = -1;
static struct sockaddr_in dest_addr;

extern "C" int udp_log_vprintf(const char *fmt, va_list args)
{
    char buffer[256];
    int len = vsnprintf(buffer, sizeof(buffer), fmt, args);

    // Send over UDP
    if (udp_sock >= 0) {
        sendto(udp_sock, buffer, len, 0,
               (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    }

    // Also print to UART (optional)
    return vprintf(fmt, args);
}

void initUdpLogger(const char *host, uint16_t port)
{
    udp_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);

    dest_addr.sin_addr.s_addr = inet_addr(host);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(port);

    // Install custom logger
    esp_log_set_vprintf(udp_log_vprintf);
}

// initUdpLogger("192.168.1.50", 5140);

void consumeLogIpCfg(const can_msg_t *msg)
{
    if (msg->data_length_code < 8)
        return;

    uint8_t a = msg->data[4];
    uint8_t b = msg->data[5];
    uint8_t c = msg->data[6];
    uint8_t d = msg->data[7];

    snprintf(udpHost, sizeof(udpHost), "%u.%u.%u.%u", a, b, c, d);

    udpLoggingEnabled = true;

    /* Reconfigure immediately if WiFi is up */
    if (wifiIsConnected()) {
        wifiEnableUdpLogging(udpHost, udpPort);
    }

    ESP_LOGI("wifi_hw", "[LOG] Updated UDP log host to %s", udpHost);
}

void consumeLogPortCfg(const can_msg_t *msg)
{
    if (msg->data_length_code < 6)
        return;

    udpPort = (msg->data[4] << 8) | msg->data[5];
    udpLoggingEnabled = true;

    if (wifiIsConnected()) {
        wifiEnableUdpLogging(udpHost, udpPort);
    }

    ESP_LOGI("wifi_hw", "[LOG] Updated UDP log port to %u", udpPort);
}

typedef struct {
    uint32_t ip_be;      /* IPv4 address in big-endian network order */
    uint16_t port_be;    /* UDP port in big-endian network order */
    uint8_t  enabled;    /* 0 = off, 1 = on */
    uint8_t  reserved;   /* reserved for future use */
} udp_log_cfg_t;

void storageSaveUdpLogCfg(const udp_log_cfg_t *cfg);
bool storageLoadUdpLogCfg(udp_log_cfg_t *cfg);
