/**
 * \file
 *
 * \brief MAIN configuration.
 *
 * Copyright (c) 2016 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */

#ifndef MAIN_H_INCLUDED
#define MAIN_H_INCLUDED

#ifdef __cplusplus
extern "C" {
#endif

#include "driver/include/m2m_wifi.h"

#define YEAR        2018
#define MONTH       3
#define DAY         1
#define WEEK        12
#define HOUR        15
#define MINUTE      45
#define SECOND      0

/** Wi-Fi Settings */
#define MAIN_WLAN_SSID                    "ANF" /**< Destination SSID */
#define MAIN_WLAN_AUTH                    M2M_WIFI_SEC_WPA_PSK /**< Security manner */
#define MAIN_WLAN_PSK                     "Orcamento2014" /**< Password for Destination SSID */
#define DEVICE 10101
#define MAIN_MAC_ADDRESS                     {0x42, 0x00, 0x61, 0x43, 0xf9, 0x3f}

/** Using broadcast address for simplicity. */
#define MAIN_SERVER_PORT                    (80)

/** IP address parsing. */
#define IPV4_BYTE(val, index)               ((val >> (index * 8)) & 0xFF)

/** Send buffer of TCP socket. */
#define MAIN_PREFIX_BUFFER                  "GET /log?tmp=10&fan=243&device=23 HTTP/1.1\r\nHost: insper.herokuapp.com\r\nAccept: */*\r\n\r\n"

//#define MAIN_PREFIX_BUFFER					"Turn Led ON ! \n"


/** Weather information provider server. */
#define MAIN_SERVER_NAME                    "insper.herokuapp.com"

//#define MAIN_SERVER_NAME                    "10.103.0.41"

/** Receive buffer size. */
#define MAIN_WIFI_M2M_BUFFER_SIZE           2000

#define MAIN_HEX2ASCII(x)                   (((x) >= 10) ? (((x) - 10) + 'A') : ((x) + '0'))

static uint8_t gau8MacAddr[] = MAIN_MAC_ADDRESS;

#ifdef __cplusplus
}
#endif

#endif /* MAIN_H_INCLUDED */
