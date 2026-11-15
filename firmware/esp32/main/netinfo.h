#pragma once
#include <stddef.h>

// Current STA IPv4, or "0.0.0.0" before DHCP.
void net_sta_ip_str(char *out, size_t len);
