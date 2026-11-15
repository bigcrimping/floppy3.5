#include "netinfo.h"
#include <string.h>
#include <stdio.h>
#include "esp_netif.h"

void net_sta_ip_str(char *out, size_t len)
{
    esp_netif_ip_info_t ip = {0};
    esp_netif_t *nif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (nif && esp_netif_get_ip_info(nif, &ip) == ESP_OK && ip.ip.addr != 0)
        snprintf(out, len, IPSTR, IP2STR(&ip.ip));
    else
        strlcpy(out, "0.0.0.0", len);
}
