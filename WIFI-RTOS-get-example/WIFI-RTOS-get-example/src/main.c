




#include "asf.h"

#include "main.h"
#include "bsp/include/nm_bsp.h"
#include "driver/include/m2m_wifi.h"
#include "socket/include/socket.h"
#include <string.h>

#define STRING_EOL "\r\n"
#define STRING_HEADER "-- WINC1500 weather client example --" STRING_EOL \
                      "-- " BOARD_NAME " --" STRING_EOL                  \
                      "-- Compiled: "__DATE__                            \
                      " "__TIME__                                        \
                      " --" STRING_EOL

#define MUS_LED_PIO PIOC
#define MUS_LED_MASK (1u << 8u)

#define BUZZ_PIO PIOA
#define BUZZ_PIO_ID ID_PIOA
#define BUZZ_PIO_IDX 6u
#define BUZZ_PIO_IDX_MASK (1u << BUZZ_PIO_IDX)

#define BUT_PIO_ID ID_PIOA
#define BUT_PIO PIOA
#define BUT_PIN 11
#define BUT_PIN_MASK (1 << BUT_PIN)

/** IP address of host. */
uint32_t gu32HostIp = 0;

/** TCP client socket handlers. */
static SOCKET tcp_client_socket = -1;

/** Receive buffer definition. */
static uint8_t gau8ReceivedBuffer[MAIN_WIFI_M2M_BUFFER_SIZE] = {0};

/** Wi-Fi status variable. */
static bool gbConnectedWifi = false;

/** Get host IP status variable. */
/** Wi-Fi connection state */
static uint8_t wifi_connected;

/** Instance of HTTP client module. */
static bool gbHostIpByName = false;

/** TCP Connection status variable. */
static bool gbTcpConnection = false;

static bool gGotResponse = false;

/** Index of scan list to request scan result. */
static uint8_t scan_request_index = 0;
/** Number of APs found. */
static uint8_t num_founded_ap = 0;

/** Server host name. */
char server_host_name[30] = MAIN_SERVER_NAME;

#define TASK_WIFI_STACK_SIZE (4096 / sizeof(portSTACK_TYPE))
#define TASK_WIFI_STACK_PRIORITY (tskIDLE_PRIORITY)

#define TASK_TRIGGER_STACK_SIZE (512 / sizeof(portSTACK_TYPE))
#define TASK_TRIGGER_STACK_PRIORITY (tskIDLE_PRIORITY)

SemaphoreHandle_t xwifiSemaphore;

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
                                          signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

/**
 * \brief Called if stack overflow during execution
 */
extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
                                          signed char *pcTaskName) {
  printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
  /* If the parameters have been corrupted then inspect pxCurrentTCB to
	 * identify which task has overflowed its stack.
	 */
  for (;;) {
  }
}

/**
 * \brief This function is called by FreeRTOS idle task
 */
extern void vApplicationIdleHook(void) {
}

/**
 * \brief This function is called by FreeRTOS each tick
 */
extern void vApplicationTickHook(void) {
}

extern void vApplicationMallocFailedHook(void) {
  /* Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */

  /* Force an assert. */
  configASSERT((volatile void *)NULL);
}

static void Button1_Handler(uint32_t id, uint32_t mask) {

  xSemaphoreGiveFromISR(xwifiSemaphore, NULL);
}

/**
 * \brief Configure UART console.
 */
static void configure_console(void) {
  const usart_serial_options_t uart_serial_options = {
      .baudrate = CONF_UART_BAUDRATE,
      .charlength = CONF_UART_CHAR_LENGTH,
      .paritytype = CONF_UART_PARITY,
      .stopbits = CONF_UART_STOP_BITS,
  };

  /* Configure UART console. */
  sysclk_enable_peripheral_clock(CONSOLE_UART_ID);
  stdio_serial_init(CONF_UART, &uart_serial_options);
}

/* 
 * Check whether "cp" is a valid ascii representation
 * of an Internet address and convert to a binary address.
 * Returns 1 if the address is valid, 0 if not.
 * This replaces inet_addr, the return value from which
 * cannot distinguish between failure and a local broadcast address.
 */
/* http://www.cs.cmu.edu/afs/cs/academic/class/15213-f00/unpv12e/libfree/inet_aton.c */
int inet_aton(const char *cp, in_addr *ap) {
  int dots = 0;
  register u_long acc = 0, addr = 0;

  do {
    register char cc = *cp;

    switch (cc) {
    case '0':
    case '1':
    case '2':
    case '3':
    case '4':
    case '5':
    case '6':
    case '7':
    case '8':
    case '9':
      acc = acc * 10 + (cc - '0');
      break;

    case '.':
      if (++dots > 3) {
        return 0;
      }
      /* Fall through */

    case '\0':
      if (acc > 255) {
        return 0;
      }
      addr = addr << 8 | acc;
      acc = 0;
      break;

    default:
      return 0;
    }
  } while (*cp++);

  /* Normalize the address */
  if (dots < 3) {
    addr <<= 8 * (3 - dots);
  }

  /* Store it if requested */
  if (ap) {
    ap->s_addr = _htonl(addr);
  }

  return 1;
}

/**
 * \brief Callback function of IP address.
 *
 * \param[in] hostName Domain name.
 * \param[in] hostIp Server IP.
 *
 * \return None.
 */
static void resolve_cb(uint8_t *hostName, uint32_t hostIp) {
  gu32HostIp = hostIp;
  if (0 == (int)IPV4_BYTE(hostIp, 0) && 0 == (int)IPV4_BYTE(hostIp, 0)) {
    gethostbyname((uint8_t *)MAIN_SERVER_NAME);
    return;
  }
  gbHostIpByName = true;
  printf("resolve_cb: %s IP address is %d.%d.%d.%d\r\n\r\n", hostName,
         (int)IPV4_BYTE(hostIp, 0), (int)IPV4_BYTE(hostIp, 1),
         (int)IPV4_BYTE(hostIp, 2), (int)IPV4_BYTE(hostIp, 3));
  sprintf(server_host_name, "%d.%d.%d.%d",
          (int)IPV4_BYTE(hostIp, 0), (int)IPV4_BYTE(hostIp, 1),
          (int)IPV4_BYTE(hostIp, 2), (int)IPV4_BYTE(hostIp, 3));
}

/**
 * \brief Callback function of TCP client socket.
 *
 * \param[in] sock socket handler.
 * \param[in] u8Msg Type of Socket notification
 * \param[in] pvMsg A structure contains notification informations.
 *
 * \return None.
 */
static void socket_cb(SOCKET sock, uint8_t u8Msg, void *pvMsg) {
  char buffer[270];
  int tmp = 15;
  int fan = 25;
  /* Check for socket event on TCP socket. */
  if (sock == tcp_client_socket) {
    // consertar aqui o post!!!!!
    switch (u8Msg) {
    case SOCKET_MSG_CONNECT: {
      printf("socket_msg_connect\n");
      if (gbTcpConnection) {
        memset(gau8ReceivedBuffer, 0, sizeof(gau8ReceivedBuffer));

        tstrSocketConnectMsg *pstrConnect = (tstrSocketConnectMsg *)pvMsg;
        if (pstrConnect && pstrConnect->s8Error >= SOCK_ERR_NO_ERROR) {

          sprintf(buffer, "{\"device\":%d,\"tmp\":%d,\"fan\":%d }", DEVICE, tmp, fan);

          sprintf((char *)gau8ReceivedBuffer, "POST / HTTP/1.1\r\nHost: insper.herokuapp.com\r\nAccept: */*\r\nContent-Type: application/json\r\ncontent-length:%d\r\n\r\n%s", strlen(buffer), buffer);

          printf("%d\n", strlen(buffer));
          printf("----------------\n%s\n-----------\n", gau8ReceivedBuffer);
          printf("send \n");
          send(tcp_client_socket, gau8ReceivedBuffer, strlen((char *)gau8ReceivedBuffer), 0);

          memset(gau8ReceivedBuffer, 0, MAIN_WIFI_M2M_BUFFER_SIZE);
          recv(tcp_client_socket, &gau8ReceivedBuffer[0], MAIN_WIFI_M2M_BUFFER_SIZE, 0);
        } else {
          printf("socket_cb: connect error!\r\n");
          gbTcpConnection = false;
          close(tcp_client_socket);
          tcp_client_socket = -1;
        }
      }
    } break;

    case SOCKET_MSG_RECV: {
      printf("Socket msg recv!\n");
      char *pcIndxPtr;
      char *pcEndPtr;

      tstrSocketRecvMsg *pstrRecv = (tstrSocketRecvMsg *)pvMsg;
      if (pstrRecv && pstrRecv->s16BufferSize > 0) {
        printf("Recebi uma mensageeem!\r\n");
        printf("pstrRecv->pu8Buffer: %s\r\n", pstrRecv->pu8Buffer);
        memset(gau8ReceivedBuffer, 0, sizeof(gau8ReceivedBuffer));

        //recv(tcp_client_socket, &gau8ReceivedBuffer[0], MAIN_WIFI_M2M_BUFFER_SIZE, 0);
        gGotResponse = true;
        close(tcp_client_socket);
        tcp_client_socket = -1;
      } else {
        printf("socket_cb: recv error!\r\n");
        close(tcp_client_socket);
        tcp_client_socket = -1;
      }
    } break;

    default:
      break;
    }
  }
}

static void set_dev_name_to_mac(uint8_t *name, uint8_t *mac_addr) {
  /* Name must be in the format WINC1500_00:00 */
  uint16 len;

  len = m2m_strlen(name);
  if (len >= 5) {
    name[len - 1] = MAIN_HEX2ASCII((mac_addr[5] >> 0) & 0x0f);
    name[len - 2] = MAIN_HEX2ASCII((mac_addr[5] >> 4) & 0x0f);
    name[len - 4] = MAIN_HEX2ASCII((mac_addr[4] >> 0) & 0x0f);
    name[len - 5] = MAIN_HEX2ASCII((mac_addr[4] >> 4) & 0x0f);
  }
}

/**
 * \brief Callback to get the Wi-Fi status update.
 *
 * \param[in] u8MsgType Type of Wi-Fi notification.
 * \param[in] pvMsg A pointer to a buffer containing the notification parameters.
 *
 * \return None.
 */
static void wifi_cb(uint8_t u8MsgType, void *pvMsg) {
  switch (u8MsgType) {
  case M2M_WIFI_RESP_CON_STATE_CHANGED: {
    tstrM2mWifiStateChanged *pstrWifiState = (tstrM2mWifiStateChanged *)pvMsg;
    if (pstrWifiState->u8CurrState == M2M_WIFI_CONNECTED) {
      printf("wifi_cb: M2M_WIFI_CONNECTED\r\n");
      m2m_wifi_request_dhcp_client();
    } else if (pstrWifiState->u8CurrState == M2M_WIFI_DISCONNECTED) {
      printf("wifi_cb: M2M_WIFI_DISCONNECTED\r\n");
      gbConnectedWifi = false;
      wifi_connected = 0;
    }

    break;
  }
  case M2M_WIFI_RESP_SCAN_DONE: {
    tstrM2mScanDone *pstrInfo = (tstrM2mScanDone *)pvMsg;
    scan_request_index = 0;
    if (pstrInfo->u8NumofCh >= 1) {
      m2m_wifi_req_scan_result(scan_request_index);
      scan_request_index++;
    } else {
      m2m_wifi_request_scan(M2M_WIFI_CH_ALL);
    }

    break;
  }

  case M2M_WIFI_RESP_SCAN_RESULT: {
    tstrM2mWifiscanResult *pstrScanResult = (tstrM2mWifiscanResult *)pvMsg;
    uint16_t demo_ssid_len;
    uint16_t scan_ssid_len = strlen((const char *)pstrScanResult->au8SSID);

    /* display founded AP. */
    printf("[%d] SSID:%s\r\n", scan_request_index, pstrScanResult->au8SSID);

    num_founded_ap = m2m_wifi_get_num_ap_found();
    if (scan_ssid_len) {
      /* check same SSID. */
      demo_ssid_len = strlen((const char *)MAIN_WLAN_SSID);
      if (
          (demo_ssid_len == scan_ssid_len) &&
          (!memcmp(pstrScanResult->au8SSID, (uint8_t *)MAIN_WLAN_SSID, demo_ssid_len))) {
        /* A scan result matches an entry in the preferred AP List.
				 * Initiate a connection request.
				 */
        printf("Found %s \r\n", MAIN_WLAN_SSID);
        m2m_wifi_connect((char *)MAIN_WLAN_SSID,
                         sizeof(MAIN_WLAN_SSID),
                         MAIN_WLAN_AUTH,
                         (void *)MAIN_WLAN_PSK,
                         M2M_WIFI_CH_ALL);
        break;
      }
    }

    if (scan_request_index < num_founded_ap) {
      m2m_wifi_req_scan_result(scan_request_index);
      scan_request_index++;
    } else {
      printf("can not find AP %s\r\n", MAIN_WLAN_SSID);
      m2m_wifi_request_scan(M2M_WIFI_CH_ALL);
    }

    break;
  }

  case M2M_WIFI_REQ_DHCP_CONF: {
    uint8_t *pu8IPAddress = (uint8_t *)pvMsg;
    printf("wifi_cb: IP address is %u.%u.%u.%u\r\n",
           pu8IPAddress[0], pu8IPAddress[1], pu8IPAddress[2], pu8IPAddress[3]);
    wifi_connected = M2M_WIFI_CONNECTED;

    gethostbyname((uint8_t *)server_host_name);
    break;
  }

  case M2M_WIFI_RESP_GET_SYS_TIME:
    /*Initial first callback will be provided by the WINC itself on the first communication with NTP */
    {
      tstrSystemTime *strSysTime_now = (tstrSystemTime *)pvMsg;

      /* Print the hour, minute and second.
			* GMT is the time at Greenwich Meridian.
			*/
      rtc_set_date(RTC, strSysTime_now->u16Year,
                   strSysTime_now->u8Month, DAY, WEEK);
      rtc_set_time(RTC,
                   strSysTime_now->u8Hour,
                   strSysTime_now->u8Minute,
                   strSysTime_now->u8Second);
      printf("socket_cb: Year: %d, Month: %d, The GMT time is %u:%02u:%02u\r\n",
             strSysTime_now->u16Year,
             strSysTime_now->u8Month,
             strSysTime_now->u8Hour,    /* hour (86400 equals secs per day) */
             strSysTime_now->u8Minute,  /* minute (3600 equals secs per minute) */
             strSysTime_now->u8Second); /* second */
      break;
    }

  default: {
    break;
  }
  }
}

/**
 * \brief This task, when activated, send every ten seconds on debug UART
 * the whole report of free heap and total tasks status
 */
static void task_monitor(void *pvParameters) {
  static portCHAR szList[256];
  UNUSED(pvParameters);

  for (;;) {
    printf("--- task ## %u", (unsigned int)uxTaskGetNumberOfTasks());
    vTaskList((signed portCHAR *)szList);
    printf(szList);
    vTaskDelay(1000);
  }
}

void buzz(int frequency, int duration) {
  double i = duration * 1000;
  long delay = 1000000.0 / frequency;
  if (frequency != 0) {
    pio_clear(PIOC, MUS_LED_MASK);
    while (i > 0) {
      pio_clear(PIOA, BUZZ_PIO_IDX_MASK);
      delay_us(delay);
      pio_set(PIOA, BUZZ_PIO_IDX_MASK);
      delay_us(delay);
      i -= delay * 2;
    }
    pio_set(PIOC, MUS_LED_MASK);
  } else {
    delay_ms(duration);
  }
}

static void task_wifi(void *pvParameters) {
  tstrWifiInitParam param;
  int8_t ret;
  uint8_t mac_addr[6];
  uint8_t u8IsMacAddrValid;

  /* Initialize the BSP. */
  nm_bsp_init();

  /* Initialize Wi-Fi parameters structure. */
  memset((uint8_t *)&param, 0, sizeof(tstrWifiInitParam));

  /* Initialize Wi-Fi driver with data and status callbacks. */
  param.pfAppWifiCb = wifi_cb;
  ret = m2m_wifi_init(&param);

  if (M2M_SUCCESS != ret) {
    printf("main: m2m_wifi_init call error!(%d)\r\n", ret);
    while (1) {
    }
  }

  /* Initialize socket module. */
  socketInit();

  /* Register socket callback function. */
  registerSocketCallback(socket_cb, resolve_cb);

  //m2m_wifi_set_mac_address(gau8MacAddr);
  m2m_wifi_request_scan(M2M_WIFI_CH_ALL);

  //inet_aton(MAIN_SERVER_NAME, &addr_in.sin_addr);

  while (1) {
    m2m_wifi_handle_events(NULL);

    if (wifi_connected == M2M_WIFI_CONNECTED) {
      /* Open client socket. */
      if (tcp_client_socket < 0) {
        printf("socket init \n");
        if ((tcp_client_socket = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
          printf("main: failed to create TCP client socket error!\r\n");
          continue;
        }
        gethostbyname((uint8_t *)MAIN_SERVER_NAME);
      }
    }
  }
}

static void task_trigger(void *pvParameters) {
  struct sockaddr_in addr_in;
  xwifiSemaphore = xSemaphoreCreateBinary();
  if (xwifiSemaphore == NULL) {
    while (1) {
      printf("No semaphore 4u\r\n");
      vTaskDelay(1000);
    }
  }
  addr_in.sin_family = AF_INET;
  addr_in.sin_port = _htons(MAIN_SERVER_PORT);
  int running = 0;
  while (true) {

    if (wifi_connected == M2M_WIFI_CONNECTED && running) {
      //Tem o IP e o socket
      if (gbHostIpByName && (tcp_client_socket >= 0)) {
        /* Connect server */
        printf("socket connecting\n\n");
        inet_aton(server_host_name, &addr_in.sin_addr);
        //printf("Inet aton : %d\n", addr_in.sin_addr);
        if (connect(tcp_client_socket, (struct sockaddr *)&addr_in, sizeof(struct sockaddr_in)) == SOCK_ERR_NO_ERROR) {
          printf("Ae, deu certo, TCP socket started\n");
          gbTcpConnection = true;
          running = 0;
          //puhh puuh puuh PUAAAHH
          buzz(500, 500);
          vTaskDelay(500);
          buzz(500, 500);
          vTaskDelay(500);
          buzz(500, 500);
          vTaskDelay(500);
          buzz(1000, 1000);
          //LIGA O LED VERDE RAPAH

        } else {
          close(tcp_client_socket);
          tcp_client_socket = -1;
          printf("error connecting socket.\n");
        }
      } else if (!gbHostIpByName) {
        printf("Problema: N temos o hostByname\n");

      } else {
        printf("Problema: N temos socket: %d\n", tcp_client_socket);
      }
      //Wifi not connected
    } else if (running) {
      printf("Scanning networks...\n");
      vTaskDelay(2000);
    } else {
      if (xSemaphoreTake(xwifiSemaphore, (TickType_t)0) == pdTRUE) {
        running = 1;
        printf("got semaphore\r\n");
      }
      running = 1;
      vTaskDelay(2000);
    }

    //DESLIGA O LED VERDE RAPAH
  }
}

void RTC_init() {
  /* Configura o PMC */
  pmc_enable_periph_clk(ID_RTC);

  /* Default RTC configuration, 24-hour mode */
  rtc_set_hour_mode(RTC, 0);

  /* Configura data e hora manualmente */
  rtc_set_date(RTC, YEAR, MONTH, DAY, WEEK);
  rtc_set_time(RTC, HOUR, MINUTE, SECOND);

  /* Configure RTC interrupts */
  //NVIC_DisableIRQ(RTC_IRQn);
  //NVIC_ClearPendingIRQ(RTC_IRQn);
  //NVIC_SetPriority(RTC_IRQn, 0);
  //NVIC_EnableIRQ(RTC_IRQn);

  /* Ativa interrupcao via alarme */
  //rtc_enable_interrupt(RTC,  RTC_IER_ALREN);
}

void Input_init(void) {
  /* config. pino botao em modo de entrada */
  pmc_enable_periph_clk(BUT_PIO);
  pio_set_input(BUT_PIO, BUT_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);

  /* config. interrupcao em borda de descida no botao do kit */
  /* indica funcao (but_Handler) a ser chamada quando houver uma interrup??o */
  pio_enable_interrupt(BUT_PIO, BUT_PIN_MASK);
  pio_handler_set(BUT_PIO, BUT_PIO_ID, BUT_PIN_MASK, PIO_IT_FALL_EDGE, Button1_Handler);

  /* habilita interrup?c?o do PIO que controla o botao */
  /* e configura sua prioridade                        */
  NVIC_EnableIRQ(BUT_PIO_ID);
  NVIC_SetPriority(BUT_PIO_ID, 4);
};
/**
 * \brief Main application function.
 *
 * Initialize system, UART console, network then start weather client.
 *
 * \return Program return value.
 */
int main(void) {
  /* Initialize the board. */
  sysclk_init();
  board_init();
  RTC_init();
  //Input_init();
  /* Initialize the UART console. */
  configure_console();
  printf(STRING_HEADER);

  if (xTaskCreate(task_wifi, "Wifi", TASK_WIFI_STACK_SIZE, NULL,
                  TASK_WIFI_STACK_PRIORITY, NULL) != pdPASS) {
    printf("Failed to create Wifi task\r\n");
  }

  if (xTaskCreate(task_trigger, "trigger", TASK_WIFI_STACK_SIZE, NULL, TASK_WIFI_STACK_PRIORITY, NULL) != pdPASS) {
    printf("Failed to create Trigger Task");
  }

  vTaskStartScheduler();

  while (1) {
  };
  return 0;
}
