/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "lwip.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "lwip/udp.h"
#include "lwip/dns.h"
#include "lwip/pbuf.h"
#include "lwip/ip_addr.h"
#include "lwip/netif.h"
#include "lwip/inet.h"
#include "FreeRTOS.h"
#include <math.h>
#include "lwip/tcp.h"
#include "task.h"
#define JSMN_PARENT_LINKS
#include "jsmn.h"
extern struct netif gnetif;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */
#define LISTEN_PORT   12345
#define MAX_MSG_LEN 128
#define NTP_SERVER "pool.ntp.org"
#define NTP_PORT 123
#define NTP_TO_UNIX 2208988800UL
#define NTP_PACKET_SIZE 48
#define MAX_NODES 10
#define FRAM_NODE_TABLE_BASE   0x000000u
#define FRAM_SLOT_SIZE         256u
#define NODE_ID_SIZE 20
#define NODE_PREFIX "nucleo"
#define NODE_PREFIX_LEN (sizeof(NODE_PREFIX) - 1)
#define SET_NODE_COUNT(v) do { node_count = (v); log_message("node_count=%lu @%s:%d\r\n",(unsigned long)node_count,__FILE__,__LINE__);} while(0)
#define INC_NODE_COUNT()  do { node_count++;        log_message("node_count=%lu @%s:%d\r\n",(unsigned long)node_count,__FILE__,__LINE__);} while(0)


/* Global variables */
volatile bool buttonPressed = false;

/* ADC/DMA global variables */
volatile uint8_t flagConversion=0;
uint16_t rawADC [30];
float rawADC_x[10], rawADC_y[10], rawADC_z[10];
float meanADCx, meanADCy, meanADCz = 0.0f;
float sqrtX, sqrtY, sqrtZ = 0.0f;
float prev_rmsX, prev_rmsY, prev_rmsZ = 0.0f;
float topLocalRMS [10][3];
float meanBufX[100], meanBufY[100], meanBufZ[100]; // To store mean vals (10 vals for 1s window)
int meanIndex = 0;
const char* currentStatus;
bool triggerAlarm = false;
static ip_addr_t ntp_ip;
bool alertStatus = false;


/* Network variables */
typedef struct {
    char id[NODE_ID_SIZE];
    ip_addr_t ip;
    float topRMS[10][3];
} NodeInfo;

const char node_id[NODE_ID_SIZE] = "nucleo-6";
static NodeInfo nodes[MAX_NODES];
static uint32_t node_count = 0;

static ip_addr_t ntp_ip;
static volatile uint8_t dns_done = 0;
static volatile err_t dns_result = ERR_VAL;

/* Timestamp variables*/
volatile uint16_t year;
volatile uint8_t  month, day;
volatile uint8_t  hour, min, sec;
volatile uint8_t ntp_time_ok = 0;

/* UART debug structure */
typedef struct {
  uint32_t id;
  char text[MAX_MSG_LEN];
} Message_t;

/* General Task Handling */
osThreadId masterTaskHandle; // Push button runs/stops specified tasks
osThreadId heartbeatTaskHandle;
osThreadId LogMessageTaskHandle;
osMailQId logMailQId;
osMutexId framMutexHandle;

/* Data Acquisition Task Handling */
osThreadId acquisitionTaskHandle; // Store raw data AND average over 10 sample per axis
osThreadId DetectionTaskHandle; // Calculate RMS vals, mean vals etc.

/* RTC Time Synchronization */
osThreadId syncRTCFromNTPTaskHandle;
osThreadId readTimeFromBQ32000TaskHandle; // store global time

/* Communication client/server Task Handling */
osThreadId TCPClientDataSyncTaskHandle; // exchange remote RMS once every 60s to keep data fresh: timestamp, rms, peak => framrecord struct type 10 max
osThreadId presenceBroadcastTaskHandle; // sends broadcast JSON msg (node alive) every 10s
osThreadId UDPServerTaskHandle; // listen for incoming UDP messages
osThreadId TCPServerTaskHandle; // Listen for incoming TCP connections for data requests

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
/* --- Functions --- */
extern void log_message(const char *format, ...);
void updateTopRMS(float table[10][3],float rms_x, float rms_y, float rms_z);
void computeRMS();
void handle_presence(const char *json);
void handle_data_request(struct tcp_pcb *pcb);
void handle_data_response(const char *json);
void handle_alert(const char *json);
err_t tcp_accept_cb(void *arg, struct tcp_pcb *newpcb, err_t err);
err_t tcp_recv_cb(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err);
static err_t tcp_client_connected(void *arg, struct tcp_pcb *pcb, err_t err);
const char* determineADCDataStatus(float x, float y, float z);
static uint8_t Dec_To_BCD(uint8_t dec);
static uint8_t BCD_To_Dec(uint8_t bcd);
static void dns_cb(const char *name, const ip_addr_t *ipaddr, void *callback_arg);
static void ntp_recv_cb(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port);

/* --- General tasks --- */
void StartMasterTask(void const * argument);
void StartHeartBeatTask(void const * argument);
void LogMessageTask(void const * argument);

/* --- Acquisition / detection / alarm --- */
void StartAcquisitionTask(void const * argument);
void StartDetectionTask(void const * argument);

/* --- RTC / NTP --- */
void StartSyncRTCFromNTPTask(void const * argument);
void StartReadTimeFromBQ32000Task(void const * argument);

/* --- Network --- */
void StartTCPClientDataSyncTask(void const * argument);
void StartPresenceBroadcastTask(void const * argument);
void StartUDPServerTask(void const * argument);
void StartTCPServerTask(void const * argument);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static int jsoneq(const char *json, jsmntok_t *tok, const char *s) {
  if (tok->type == JSMN_STRING && (int)strlen(s) == tok->end - tok->start &&
      strncmp(json + tok->start, s, tok->end - tok->start) == 0) {
    return 0;
  }
  return -1;
}

/* ============================= Function Callbacks ===============================*/


static void dns_cb(const char *name, const ip_addr_t *ipaddr, void *callback_arg)
{
    if (ipaddr != NULL) {
        ntp_ip = *ipaddr;
        dns_result = ERR_OK;
    }
    else {
        dns_result = ERR_VAL;
    }
    dns_done = 1;
}

static void ntp_recv_cb(void *arg, struct udp_pcb *pcb, struct pbuf *p,
                        const ip_addr_t *addr, u16_t port)
{
    uint8_t buf[48];

    if (!p) return;

    if (p->tot_len < 48) {
        pbuf_free(p);
        return;
    }

    if (pbuf_copy_partial(p, buf, 48, 0) != 48) {
        pbuf_free(p);
        return;
    }

    pbuf_free(p);

    uint32_t ntp_seconds = ((uint32_t)buf[40] << 24) | ((uint32_t)buf[41] << 16) |((uint32_t)buf[42] << 8)  | ((uint32_t)buf[43]);

    if (ntp_seconds < NTP_TO_UNIX) {
        log_message("NTP: invalid time");
    } else {
        uint32_t unix_time = ntp_seconds - NTP_TO_UNIX;
        log_message("NTP unix time: %lu", (unsigned long)unix_time);
        uint32_t belgium_time = unix_time + 3600; // winter + 3600
        sec  = belgium_time % 60;
        belgium_time /= 60;
        min  = belgium_time % 60;
        belgium_time /= 60;
        hour = belgium_time % 24;
        ntp_time_ok = 1;
    }

    udp_remove(pcb);
}


static err_t tcp_client_connected(void *arg, struct tcp_pcb *pcb, err_t err)
{
	// loop node count times
    if (err != ERR_OK) {
        tcp_close(pcb);
        return err;
    }

    const char *msg = "{\"type\":\"data_request\"}";
    tcp_write(pcb, msg, strlen(msg), TCP_WRITE_FLAG_COPY);
    tcp_output(pcb);

    tcp_recv(pcb, tcp_recv_cb); // Data_response will be sent receive callback

    return ERR_OK;
}

void udp_receive_callback(void *arg, struct udp_pcb *pcb,struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
	char jsonPayload[256];
	if (!p) return;

	u16_t len = p->tot_len;
	if (len >= sizeof(jsonPayload)) len = sizeof(jsonPayload) - 1;
	pbuf_copy_partial(p, jsonPayload, len, 0);
	jsonPayload[len] = '\0';

    log_message("\n\n======> Received payload <======\r\n: %s\r\n", jsonPayload);

	// Check json type payload
    if (strstr(jsonPayload, "presence")) {
        handle_presence(jsonPayload);
    }
    else if (strstr(jsonPayload, "alert")) {
        handle_alert(jsonPayload);
    }
	else{
		log_message("JSON payload invalid type\r\n");
	}
	pbuf_free(p);
}

err_t tcp_accept_cb(void *arg, struct tcp_pcb *newpcb, err_t err)
{
    LWIP_UNUSED_ARG(arg);
    if (err != ERR_OK || newpcb == NULL) {
        return ERR_VAL;
    }
    tcp_recv(newpcb, tcp_recv_cb);

    log_message("TCP: client connected");
    return ERR_OK;
}


err_t tcp_recv_cb(void *arg, struct tcp_pcb *pcb,struct pbuf *p, err_t err)
{
    if (err != ERR_OK)
    {
    	if (p) pbuf_free(p);
        tcp_close(pcb);
        return err;
    }

    if (p == NULL) {
        tcp_close(pcb);
        return ERR_OK;
    }

    char jsonPayload[256];
    u16_t len = p->tot_len;
	if (len >= sizeof(jsonPayload)) len = sizeof(jsonPayload) - 1;
	pbuf_copy_partial(p, jsonPayload, len, 0);
	jsonPayload[len] = '\0';

    log_message("\n\n======> Received payload <======\r\n: %s\r\n", jsonPayload);

    // Tell TCP stack we consumed data
    tcp_recved(pcb, p->tot_len);
    pbuf_free(p);

    if (strstr(jsonPayload, "data_request")){
        handle_data_request(pcb);
        return ERR_OK;
    }
    else if (strstr(jsonPayload, "data_response")){
    	handle_data_response(jsonPayload);
		return ERR_OK;
    }
    else{
		log_message("JSON payload invalid type\r\n");
	}
    tcp_close(pcb);
    return ERR_OK;
}


/* =========================== Handle functions ========================*/
void handle_presence(const char *json){
	char id_buf[20];
	char ip_buf[32];
	jsmn_parser p;
	jsmntok_t t[16];

	jsmn_init(&p);
	int r = jsmn_parse(&p, json, strlen(json), t, 16);
	if (r < 0) return;

	for (int i = 1; i < r; i++) {
		if (jsoneq(json, &t[i], "id") == 0) {
			int len = t[i+1].end - t[i+1].start;
			memcpy(id_buf, json + t[i+1].start, len);
			id_buf[len] = '\0';
			i++;
		}
		if (jsoneq(json, &t[i], "ip") == 0) {
			int len = t[i+1].end - t[i+1].start;
			memcpy(ip_buf, json + t[i+1].start, len);
			ip_buf[len] = '\0';
			i++;
		}
	}

	bool id_exist = false;
	for (uint32_t i = 0; i < node_count; i++) {
	    if (strncmp(nodes[i].id, id_buf, NODE_ID_SIZE) == 0) {
	        id_exist = true;
	        break;
	    }
	}


	if(!id_exist && (node_count < MAX_NODES)){
		NodeInfo new_node;

		ipaddr_aton(ip_buf, &new_node.ip); // Convert str to ipaddress
		strncpy(new_node.id, id_buf, sizeof(new_node.id));

		nodes[node_count]=new_node;
		node_count++;
		log_message("New presence received, storing IP & ID in local memory : \r\nIP= %s\r\nID= %s",ip_buf, id_buf);
		/*
		 * When a node sends presence (new), saves locally ID with empty topRMS array with struct
		 * Restores any old data from FRAM if node already wrote some data before
		 */
		restoringDataFromFRAMForAliveNode(id_buf);
	}
	else{
		log_message("Old presence received, already stored in local memory : \r\nIP= %s\r\nID= %s",ip_buf, id_buf);
	}

}

void handle_data_request(struct tcp_pcb *pcb)
{
    char response[200];
    char timestamp[32];
    char type[]  = "data_response";
    const char* status = determineADCDataStatus(sqrtX,sqrtY,sqrtZ);

    snprintf(timestamp, sizeof(timestamp), "2000-00-0000:00:00Z");

    int resp_len = snprintf(response, sizeof(response),
                            "{"
                              "\"type\":\"%s\","
                              "\"id\":\"%s\","
                              "\"timestamp\":\"%s\","
                              "\"acceleration\":{"
                                "\"x\":%.4f,"
                                "\"y\":%.4f,"
                                "\"z\":%.4f"
                              "},"
                              "\"Status\":\"%s\""
                            "}",
                            type, node_id, timestamp,
							sqrtX, sqrtY, sqrtZ,
                            status);

    err_t err = tcp_write(pcb, response, resp_len, TCP_WRITE_FLAG_COPY);
    if (err != ERR_OK) {
        log_message("tcp_write error: %d", err);
        tcp_close(pcb);
        return;
    }

    err = tcp_output(pcb);
    log_message("TCP data_response sent (err=%d)", err);
    tcp_close(pcb);
}



void handle_data_response(const char *json){
	// Extract remote RMS vals and store in data struct NodeInfo (matrix 10x3)
	// This data_response will be called x nodes times (client request loop)
	static uint8_t nodeAlertCount = 0;
	static uint8_t nodeCountTrack = 0;
	float rms_x = 0.0f;
	float rms_y = 0.0f;
	float rms_z = 0.0f;
	jsmn_parser p;
	jsmntok_t t [36];
	char status [16];
	char nucleoID [16];

	jsmn_init(&p);
	int r = jsmn_parse(&p, json, strlen(json), t, 36); // json item count
	if (r < 0) return;

	for (int i = 1; i < r; i++) {
		if (jsoneq(json, &t[i], "acceleration") == 0) {
			int selected_obj = i + 1;
			int j = selected_obj + 1;

			while (j < r && t[j].parent == selected_obj) {
				if (jsoneq(json, &t[j], "x") == 0) {
					char buf[16];
					int len = t[j+1].end - t[j+1].start;
					memcpy(buf, json + t[j+1].start, len);
					buf[len] = '\0';
					rms_x = atof(buf);
					j += 2;
				}
				else if (jsoneq(json, &t[j], "y") == 0) {
					char buf[16];
					int len = t[j+1].end - t[j+1].start;
					memcpy(buf, json + t[j+1].start, len);
					buf[len] = '\0';
					rms_y = atof(buf);
					j += 2;
				}
				else if (jsoneq(json, &t[j], "z") == 0) {
					char buf[16];
					int len = t[j+1].end - t[j+1].start;
					memcpy(buf, json + t[j+1].start, len);
					buf[len] = '\0';
					rms_z = atof(buf);
					j += 2;
				}
			}
		}
		if (jsoneq(json, &t[i], "status") == 0) {
			char buf[16];
			int len = t[i+1].end - t[i+1].start;
			memcpy(buf, json + t[i+1].start, len);
			buf[len] = '\0';
			strcpy(status,buf);
			i++;
		}
		if (jsoneq(json, &t[i], "id") == 0) {
			char buf[16];
			int len = t[i+1].end - t[i+1].start;
			memcpy(buf, json + t[i+1].start, len);
			buf[len] = '\0';
			strcpy(nucleoID,buf);
			i++;
		}
	}
	log_message("Data_response from %s: rms=(%.4f, %.4f, %.4f) status=%s",nucleoID, rms_x, rms_y, rms_z, status);
	// Store remote RMS values to struct
	for(int i=0;i<node_count;i++){
		if(strstr(nucleoID,nodes[i].id)){
			updateTopRMS(nodes[i].topRMS,rms_x, rms_y, rms_z);
			log_message("======= TOP 10 REMOTE RMS VALUES of %s =======",nodes[i].id);
			for (int j = 0; j < 10; j++)
			{
				log_message("#%02d:  X=%.4f   Y=%.4f   Z=%.4f",
							j,
							nodes[i].topRMS[j][0],
							nodes[i].topRMS[j][1],
							nodes[i].topRMS[j][2]);
			}
			log_message("========================================");
			nodeCountTrack++;
			log_message("========> Number of nodes sent DATA request : %d (Known nodes count = %d\r\n",nodeCountTrack,node_count);
			updateTableToFRAM(nucleoID); // Update current node local data to FRAM
			if(strcmp(status,"alert")==0){
				nodeAlertCount++;
			}
			break;
		}
	}

	// If all nodes return "alert" status & current status also, then trigger alarm
	if(node_count == nodeCountTrack){
		nodeCountTrack = 0;
		nodeAlertCount = 0;
		if( node_count == nodeAlertCount && strcmp(currentStatus,"alert") == 0 ){ // all nodes were looped through and sent "alert" status
			log_message("\n ======================================== \r\n");
			log_message("\n\n All nodes send an alert status! \r\n");
			triggerAlarm = true;
		}
		else{
			log_message("\n ======================================== \r\n");
			log_message("\n\n STATUS NORMAL OVERALL \r\n");
			triggerAlarm = false;
		}
	}
}

void handle_alert(const char *json){

}

const char* determineADCDataStatus(float x, float y, float z)
{
	float warning_sumRMS = 1.6f;
	float alert_sumRMS = 1.75f;
	float sumRMS_current = 0.0f;

	sumRMS_current = (x + y + z)/3;


	log_message("========> Sum computed : %.4f",sumRMS_current);

	if(sumRMS_current < warning_sumRMS){
		return "normal";
	}
	else if(sumRMS_current >= warning_sumRMS && sumRMS_current < alert_sumRMS){
		return "warning";
	}
	else{
		alertStatus = true;
		return "alert";
	}

}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim2);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)rawADC, 30);
  HAL_UART_Transmit(&huart3, (uint8_t*)"SYSTEM BOOT...\r\n", strlen("SYSTEM BOOT...\r\n"), HAL_MAX_DELAY);
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 256);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ...
   * osThreadDef(name, function, priority, instances, stack_size);
   * osThreadId handle = osThreadCreate(osThread(name), NULL);
   * */
  /* ========================= RTOS TASKS CREATION ========================= */

  /* Stack size for each task is sized depending on Highwatermark value to leave at least 30% free memory on the stack size */
  osMailQDef(logMailQ, 32, Message_t);
  logMailQId = osMailCreate(osMailQ(logMailQ), NULL);

  osMutexDef(framMutex);
  framMutexHandle = osMutexCreate(osMutex(framMutex));

  /* --- General tasks --- */
  osThreadDef(masterTask, StartMasterTask, osPriorityNormal, 0, 256);
  masterTaskHandle = osThreadCreate(osThread(masterTask), NULL);

  osThreadDef(heartbeatTask, StartHeartBeatTask, osPriorityBelowNormal, 0, 128);
  heartbeatTaskHandle = osThreadCreate(osThread(heartbeatTask), NULL);

  osThreadDef(LogMessageTask, LogMessageTask, osPriorityNormal, 0, 256);
  LogMessageTaskHandle = osThreadCreate(osThread(LogMessageTask), NULL);


  /* --- Acquisition / detection / alarm --- */
  osThreadDef(acquisitionTask, StartAcquisitionTask, osPriorityNormal, 0, 256);
  acquisitionTaskHandle = osThreadCreate(osThread(acquisitionTask), NULL);

  osThreadDef(DetectionTask, StartDetectionTask, osPriorityNormal, 0, 128);
  DetectionTaskHandle = osThreadCreate(osThread(DetectionTask), NULL);


  /* --- RTC / NTP time sync --- */
  osThreadDef(syncRTCFromNTPTask, StartSyncRTCFromNTPTask, osPriorityBelowNormal, 0, 256);
  syncRTCFromNTPTaskHandle = osThreadCreate(osThread(syncRTCFromNTPTask), NULL);

  osThreadDef(readTimeFromBQ32000Task, StartReadTimeFromBQ32000Task, osPriorityBelowNormal, 0, 256);
  readTimeFromBQ32000TaskHandle = osThreadCreate(osThread(readTimeFromBQ32000Task), NULL);

  /* --- Network communication tasks --- */
  osThreadDef(TCPClientDataSyncTask, StartTCPClientDataSyncTask,osPriorityBelowNormal, 0, 256);
  TCPClientDataSyncTaskHandle = osThreadCreate(osThread(TCPClientDataSyncTask), NULL);

  osThreadDef(presenceBroadcastTask, StartPresenceBroadcastTask,osPriorityBelowNormal, 0, 256);
  presenceBroadcastTaskHandle = osThreadCreate(osThread(presenceBroadcastTask), NULL);

  osThreadDef(UDPServerTask, StartUDPServerTask,osPriorityAboveNormal, 0, 256);
  UDPServerTaskHandle = osThreadCreate(osThread(UDPServerTask), NULL);

  osThreadDef(TCPServerTask, StartTCPServerTask, osPriorityNormal, 0, 256);
  TCPServerTaskHandle = osThreadCreate(osThread(TCPServerTask), NULL);


  /* ====================================================================== */
  extern size_t xPortGetFreeHeapSize(void);
  extern size_t xPortGetMinimumEverFreeHeapSize(void);

  char buf[64];
  snprintf(buf, sizeof(buf), "Free heap before scheduler: %u bytes\r\n",
           (unsigned)xPortGetFreeHeapSize());
  HAL_UART_Transmit(&huart3, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);

  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x20303E5D;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 95;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream4_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, HeartbeatLED_Pin|AlarmLED_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : HeartbeatLED_Pin AlarmLED_Pin LD2_Pin */
  GPIO_InitStruct.Pin = HeartbeatLED_Pin|AlarmLED_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI2_CS_Pin */
  GPIO_InitStruct.Pin = SPI2_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI2_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* ======================= USER FUNCTION DEFINITIONS ======================= */
void print_node(const NodeInfo *node, int index)
{
    if (node == NULL) return;

    log_message("Node [%d]:", index);
    log_message("  IP  : %s", ipaddr_ntoa(&node->ip));
    log_message("  ID  : %s", node->id);

}

void print_all_nodes(void)
{
    log_message("=== Nodes table (count = %d) ===", node_count);

    for (int i = 0; i < node_count; i++) {
        print_node(&nodes[i], i);
    }

    log_message("=== End of nodes table ===");
}


static uint8_t BCD_To_Dec(uint8_t bcd)
{
    return ((bcd >> 4) * 10) + (bcd & 0x0F);
}

static uint8_t Dec_To_BCD(uint8_t dec)
{
    return ((dec / 10) << 4) | (dec % 10);
}


/* Every data sync routine call, TOP RMS values are updated from all nodes to RMS array struct ==> Update these data from FRAM
 FRAM addressing : x bits block reserved for each node => node_ID top 10 rms_x top 10 rms_y top 10 rms_z ; node_ID etc....
 ========= Total Size per slot calculation
 * NodeID [20] = 20 bytes
 * float topRMS [10][3] = 30*4 = 120 bytes
 * CRC32 = 4 bytes
 * Total per slot ID: 144 bytes => round to 2^8 => 256 bytes per slot
 */
void restoringDataFromFRAMForAliveNode(const char *nodeID)
{
    osMutexWait(framMutexHandle, osWaitForever);

    char fetched_nodeID[NODE_ID_SIZE + 1];
    float topRMS[10][3];
    HAL_StatusTypeDef status;

    log_message("============= > Restoring data for %s", nodeID);

    for (int slot = 0; slot < MAX_NODES; slot++) {
        uint32_t addr = FRAM_NODE_TABLE_BASE + (uint32_t)slot * FRAM_SLOT_SIZE;

        uint8_t cmd[4] = {
            0x03,
            (addr >> 16) & 0xFF,
            (addr >>  8) & 0xFF,
            (addr >>  0) & 0xFF
        };

        HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);
        status = HAL_SPI_Transmit(&hspi2, cmd, sizeof(cmd), HAL_MAX_DELAY);
        if (status != HAL_OK) goto spi_error;
        status = HAL_SPI_Receive(&hspi2, (uint8_t*)fetched_nodeID, NODE_ID_SIZE, HAL_MAX_DELAY);
        if (status != HAL_OK) goto spi_error;
        HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);

        fetched_nodeID[NODE_ID_SIZE] = '\0';

        if (strncmp(fetched_nodeID, nodeID, NODE_ID_SIZE) == 0) {

            log_message("%s found in FRAM memory ! Fetching data...", nodeID);

            addr += NODE_ID_SIZE;
            cmd[1] = (addr >> 16) & 0xFF;
            cmd[2] = (addr >>  8) & 0xFF;
            cmd[3] = (addr >>  0) & 0xFF;

            HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);
            status = HAL_SPI_Transmit(&hspi2, cmd, sizeof(cmd), HAL_MAX_DELAY);
            if (status != HAL_OK) goto spi_error;
            status = HAL_SPI_Receive(&hspi2, (uint8_t*)topRMS, sizeof(topRMS), HAL_MAX_DELAY);
            if (status != HAL_OK) goto spi_error;
            HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);

            if (strncmp(nodeID, node_id, NODE_ID_SIZE) == 0) {
                memcpy(topLocalRMS, topRMS, sizeof(topRMS));
                log_message("Restored FRAM RMS into LOCAL topLocalRMS for %s", nodeID);
                osMutexRelease(framMutexHandle);
                return;
            }

            for (uint32_t j = 0; j < node_count; j++) {
                if (strncmp(nodes[j].id, nodeID, NODE_ID_SIZE) == 0) {
                    memcpy(nodes[j].topRMS, topRMS, sizeof(topRMS));
                    log_message("Restored FRAM RMS into nodes[%lu].topRMS for %s", (unsigned long)j, nodeID);
                    osMutexRelease(framMutexHandle);
                    return;
                }
            }

            log_message("Node %s was in FRAM but not in nodes[] yet", nodeID);
            osMutexRelease(framMutexHandle);
            return;
        }
    }

    log_message("%s not found in FRAM", nodeID);
    osMutexRelease(framMutexHandle);
    return;

spi_error:
    HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);
    log_message("SPI error while restoring data");
    osMutexRelease(framMutexHandle);
}


void updateTableToFRAM(const char *nodeID){
	osMutexWait(framMutexHandle, osWaitForever);
    uint8_t wren = 0x06;
    char fetched_nodeID[NODE_ID_SIZE + 1];
    HAL_StatusTypeDef status;
    bool slotFound = false;
    uint32_t empty_addr = 0;
    float topRMS[10][3] = {0};

    if (strncmp(nodeID, node_id, NODE_ID_SIZE) == 0) {
        memcpy(topRMS, topLocalRMS, sizeof(topRMS));   // write local top10 directly
    } else {
        bool found = false;
        for (int j = 0; j < node_count; j++) {
            if (strncmp(nodes[j].id, nodeID, NODE_ID_SIZE) == 0) {
                memcpy(topRMS, nodes[j].topRMS, sizeof(topRMS));
                found = true;
                break;
            }
        }
        if (!found) {
            log_message("updateTableToFRAM: node %s not found in nodes[]; not writing junk.\r\n", nodeID);
            osMutexRelease(framMutexHandle);
            return;
        }
    }

    for (int i = 0; i < MAX_NODES; i++) {
        uint32_t addr_point = FRAM_NODE_TABLE_BASE + (uint32_t)i * FRAM_SLOT_SIZE;

        uint8_t pReadCommand[4];
        pReadCommand[0] = 0x03;
        pReadCommand[1] = (addr_point >> 16) & 0xFF;
        pReadCommand[2] = (addr_point >>  8) & 0xFF;
        pReadCommand[3] = (addr_point >>  0) & 0xFF;

        // READ node ID
        HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);
        status = HAL_SPI_Transmit(&hspi2, pReadCommand, sizeof(pReadCommand), HAL_MAX_DELAY);
        if (status != HAL_OK) goto spi_error;
        status = HAL_SPI_Receive(&hspi2, (uint8_t*)fetched_nodeID, NODE_ID_SIZE, HAL_MAX_DELAY);
        if (status != HAL_OK) goto spi_error;
        HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);

        // If fetchednodeID is empty = empty slot
        if (!slotFound && memcmp(fetched_nodeID, NODE_PREFIX, NODE_PREFIX_LEN) != 0) {
            empty_addr = addr_point;
            slotFound = true;
        }

        fetched_nodeID[NODE_ID_SIZE] = '\0';

        // if node exists -> update in place (NO duplicate)
        if (strncmp(fetched_nodeID, nodeID, NODE_ID_SIZE) == 0) {
        	log_message("Node ID already exist in FRAM memory, overwriting data...\r\n\n");
            uint32_t data_addr = addr_point + NODE_ID_SIZE;

            uint8_t pWriteCommand[4];
            pWriteCommand[0] = 0x02;
            pWriteCommand[1] = (data_addr >> 16) & 0xFF;
            pWriteCommand[2] = (data_addr >>  8) & 0xFF;
            pWriteCommand[3] = (data_addr >>  0) & 0xFF;

            // WREN
            HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);
            status = HAL_SPI_Transmit(&hspi2, &wren, 1, HAL_MAX_DELAY);
            if (status != HAL_OK) goto spi_error;
            HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);

            // WRITE topRMS
            HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);
            status = HAL_SPI_Transmit(&hspi2, pWriteCommand, sizeof(pWriteCommand), HAL_MAX_DELAY);
            if (status != HAL_OK) goto spi_error;
            status = HAL_SPI_Transmit(&hspi2, (uint8_t*)topRMS, sizeof(topRMS), HAL_MAX_DELAY);
            if (status != HAL_OK) goto spi_error;
            HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);
            osMutexRelease(framMutexHandle);
            return;
        }
    }


    // node not found -> create it in first empty slot
    if (slotFound) {
    	log_message("Node ID doesn't exist in FRAM memory, writing data to empty slot! \r\n\n");
        uint8_t pWriteCommand[4];
        pWriteCommand[0] = 0x02;
        pWriteCommand[1] = (empty_addr >> 16) & 0xFF;
        pWriteCommand[2] = (empty_addr >>  8) & 0xFF;
        pWriteCommand[3] = (empty_addr >>  0) & 0xFF;

        // WREN
        HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);
        status = HAL_SPI_Transmit(&hspi2, &wren, 1, HAL_MAX_DELAY);
        if (status != HAL_OK) goto spi_error;
        HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);

        // WRITE nodeID then topRMS
        HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);
        status = HAL_SPI_Transmit(&hspi2, pWriteCommand, sizeof(pWriteCommand), HAL_MAX_DELAY);
        if (status != HAL_OK) goto spi_error;
        status = HAL_SPI_Transmit(&hspi2, (uint8_t*)nodeID, NODE_ID_SIZE, HAL_MAX_DELAY);
        if (status != HAL_OK) goto spi_error;
        status = HAL_SPI_Transmit(&hspi2, (uint8_t*)topRMS, sizeof(topRMS), HAL_MAX_DELAY);
        if (status != HAL_OK) goto spi_error;
        HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);
        osMutexRelease(framMutexHandle);
        return;
    }

    log_message("No slots available to write to FRAM for %s\r\n", nodeID);
    osMutexRelease(framMutexHandle);
    return;

spi_error:
    HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);
    log_message("SPI error while updating FRAM\r\n");
    osMutexRelease(framMutexHandle);
    return;
}


void updateTopRMS(float table[10][3],float rms_x, float rms_y, float rms_z){
	float sqrtXYZ [3] = {rms_x,rms_y,rms_z};

	// Check if new value is higher than stored values (assuming initially stored all 0.0)
	for (int j = 0; j < 3; j++) {
		int minIndex = 0;
		for (int i = 1; i < 10; i++) {
			if (table[i][j] < table[minIndex][j]) {
				minIndex = i;
			}
		}
		if (sqrtXYZ[j] > table[minIndex][j]) {
			table[minIndex][j] = sqrtXYZ[j];
		}
	}

	log_message("======= TOP 10 LOCAL RMS VALUES =======");
	for (int i = 0; i < 10; i++)
	{
		log_message("#%02d:  X=%.4f   Y=%.4f   Z=%.4f",
					i,
					topLocalRMS[i][0],
					topLocalRMS[i][1],
					topLocalRMS[i][2]);
	}
	log_message("========================================");
}


void computeRMS(){
	// store new local top 10 rms vals to topLocalRMS data taken from meanBufX[100], meanBufY[100], meanBufZ[100];

	float sumX, sumY, sumZ = 0.0f;

	for(int i=0; i<10;i++){
		sumX += meanBufX[i] * meanBufX[i];
		sumY += meanBufY[i] * meanBufY[i];
		sumZ += meanBufZ[i] * meanBufZ[i];
	}
	sqrtX = sqrtf(sumX / 10.0f);
	sqrtY = sqrtf(sumY / 10.0f);
	sqrtZ = sqrtf(sumZ / 10.0f);
	log_message("RMS computed over 1s window: X=%.4f  Y=%.4f  Z=%.4f", sqrtX, sqrtY, sqrtZ);

	updateTopRMS(topLocalRMS, sqrtX, sqrtY, sqrtZ);
}

void log_message(const char *format, ...)
{
    Message_t *msg = osMailAlloc(logMailQId, 0); // 0 no wait time (non blocking)
    if (!msg) return;

    va_list ap;
    va_start(ap, format);
    vsnprintf(msg->text, sizeof(msg->text), format, ap);
    va_end(ap);

    osMailPut(logMailQId, msg);
}
/* ======================= USER TASK FUNCTION DEFINITIONS ======================= */

/* --- General Task Handling --- */
void StartMasterTask(void const * argument)
{
	static bool running = true;
	static bool lastButtonState = false;
	static bool heapSizeChecked = false;

    for(;;) {

    	bool currentButton = buttonPressed;


    	if(currentButton && !lastButtonState){ // If button got toggled
    		if(!running){
				vTaskResume(acquisitionTaskHandle);
				vTaskResume(DetectionTaskHandle);
				vTaskResume(heartbeatTaskHandle);
//				vTaskResume(presenceBroadcastTaskHandle);
//				vTaskResume(listenDataRequestTaskHandle);
//				vTaskResume(sendsDataMessageTaskHandle);
//				vTaskResume(UDPClientDataSyncTaskHandle);
				log_message("USER button pressed! Resuming Tasks");
				log_message("[Tick=%lu", (unsigned long)HAL_GetTick());
				running = true;
			}
    		else{
				vTaskSuspend(acquisitionTaskHandle);
				vTaskSuspend(DetectionTaskHandle);
    			vTaskSuspend(heartbeatTaskHandle);
//				vTaskSuspend(presenceBroadcastTaskHandle);
//				vTaskSuspend(listenDataRequestTaskHandle);
//				vTaskSuspend(sendsDataMessageTaskHandle);
//				vTaskSuspend(UDPClientDataSyncTaskHandle);
				log_message("USER button pressed! Suspending Tasks");
				log_message("Tick=%lu", (unsigned long)HAL_GetTick());
				running = false;
			}
    	}

    	lastButtonState = currentButton;
    	if(!heapSizeChecked){
        	UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(masterTaskHandle);
        	log_message("MasterTask min free : %lu words\r\n",uxHighWaterMark);
        	heapSizeChecked = true;
    	}

    	osDelay(1);
    }
}

void StartHeartBeatTask(void const * argument)
{
	static bool heapSizeChecked = false;

    for(;;) {
    	HAL_GPIO_TogglePin(GPIOB, HeartbeatLED_Pin);
    	if(!heapSizeChecked){
			UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(heartbeatTaskHandle);
			log_message("HeartBeatTask min free : %lu words\r\n",uxHighWaterMark);
			heapSizeChecked = true;
		}
    	osDelay(400);
    }
}

void LogMessageTask(void const * argument)
{
	static bool heapSizeChecked = false;
    for (;;)
    {
        osEvent evt = osMailGet(logMailQId, osWaitForever);
        if (evt.status == osEventMail)
        {
            Message_t *m = evt.value.p;
            HAL_UART_Transmit(&huart3, (uint8_t*)m->text, strlen(m->text), HAL_MAX_DELAY);
            HAL_UART_Transmit(&huart3, (uint8_t*)"\r\n", strlen("\r\n"), HAL_MAX_DELAY);
            osMailFree(logMailQId, m);
        }
        if(!heapSizeChecked){
			UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(LogMessageTaskHandle);
			log_message("LogMessageTask min free : %lu words\r\n",uxHighWaterMark);
			heapSizeChecked = true;
		}
        osDelay(1);
    }
}



/* --- Data Acquisition Task Handling --- */
void StartAcquisitionTask(void const * argument)
{
	static bool heapSizeChecked = false;
	//restoringDataFromFRAMForAliveNode(node_id);

    for(;;) {
    	float sumADC_x = 0.0f, sumADC_y = 0.0f, sumADC_z = 0.0f;
		if(flagConversion){ // Every 100 Hz triggered by TIM2 (10 raw vals on each axe)
			for(int i=0;i<30;i++){
				float adc =  ((float)rawADC[i] * 3.3f)/ 4095.0f;
				if(i%3 == 0){
					rawADC_x[i/3]=adc;
					sumADC_x+=adc;
				}
				else if(i%3 == 1){
					rawADC_y[i/3]=adc;
					sumADC_y+=adc;
				}
				else if(i%3 == 2){
					rawADC_z[i/3]=adc;
					sumADC_z+=adc;
				}
			}
			flagConversion=0;
			meanADCx = sumADC_x / 10.0f;
			meanADCy = sumADC_y / 10.0f;
			meanADCz = sumADC_z / 10.0f;
			meanBufX[meanIndex] = meanADCx;
			meanBufY[meanIndex] = meanADCy;
			meanBufZ[meanIndex] = meanADCz;
			meanIndex++;
			if(meanIndex==10){
				meanIndex=0; // Reset after storing 10 mean vals
				log_message("10 mean data samples acquired => computing RMS (1s window)\r\n");
				computeRMS();
				currentStatus = determineADCDataStatus(sqrtX,sqrtY,sqrtZ);
				log_message("Current Status : %s \r\n",currentStatus);
				prev_rmsX=sqrtX;
				prev_rmsY=sqrtY;
				prev_rmsZ=sqrtZ;
			}

			if(!heapSizeChecked){
				UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(acquisitionTaskHandle);
				log_message("AcquisitionTask min free : %lu words\r\n",uxHighWaterMark);
				heapSizeChecked = true;
			}
		}
		osDelay(1);
    }
}

void StartDetectionTask(void const * argument){
	for(;;){
		if(triggerAlarm)HAL_GPIO_TogglePin(GPIOB, AlarmLED_Pin);
		osDelay(400);
	}
}


/* --- RTC Time Synchronization --- */
void StartSyncRTCFromNTPTask(void const * argument)
{
    static struct udp_pcb *pcb = NULL;
    err_t err;
    err_t err_dns;

    uint8_t ntp_packet[48];

    while (!netif_is_up(&gnetif)) {
        osDelay(100);
    }

    dns_done = 0;
    dns_result = ERR_VAL;
    ip_addr_set_zero(&ntp_ip);

    err_dns = dns_gethostbyname_addrtype("be.pool.ntp.org",
                                        &ntp_ip,
                                        dns_cb,
                                        NULL,
                                        LWIP_DNS_ADDRTYPE_IPV4);

    if (err_dns == ERR_OK) {
        log_message("NTP DNS resolved : %s", ipaddr_ntoa(&ntp_ip));
    }
    else if (err_dns == ERR_INPROGRESS) {
        log_message("NTP DNS resolving (async)");

        uint32_t timeout_ms = 5000;
        while (!dns_done && timeout_ms > 0) {
            osDelay(50);
            timeout_ms -= 50;
        }

        if (!dns_done || dns_result != ERR_OK || ip_addr_isany(&ntp_ip)) {
            log_message("NTP DNS failed/timeout (res=%d)", (int)dns_result);
            vTaskDelete(NULL);
        }

        log_message("NTP DNS resolved: %s", ipaddr_ntoa(&ntp_ip));
    }
    else {
        log_message("NTP DNS start failed: %d", (int)err_dns);
        vTaskDelete(NULL);
    }

    pcb = udp_new_ip_type(IPADDR_TYPE_V4);
    if (!pcb) {
        log_message("NTP: udp_new failed");
        vTaskDelete(NULL);
    }

    err = udp_bind(pcb, IP_ADDR_ANY, 0);
    if (err != ERR_OK) {
        log_message("NTP: udp_bind failed: %d", (int)err);
        udp_remove(pcb);
        vTaskDelete(NULL);
    }

    udp_recv(pcb, ntp_recv_cb, NULL);
    log_message("UDP NTP socket ready");

    memset(ntp_packet, 0, sizeof(ntp_packet));
    ntp_packet[0] = 0x23; // LI=0, VN=4, Mode=3 (client)

    struct pbuf *pb = pbuf_alloc(PBUF_TRANSPORT, sizeof(ntp_packet), PBUF_RAM);
    if (!pb) {
        log_message("NTP: pbuf_alloc failed");
        udp_remove(pcb);
        vTaskDelete(NULL);
    }

    memcpy(pb->payload, ntp_packet, sizeof(ntp_packet));

    err = udp_sendto(pcb, pb, &ntp_ip, NTP_PORT);
    pbuf_free(pb);

    if (err != ERR_OK) {
        log_message("NTP: udp_sendto failed: %d", (int)err);
        udp_remove(pcb);
        vTaskDelete(NULL);
    }

    log_message("NTP request sent");
    vTaskDelete(NULL);
}


void StartReadTimeFromBQ32000Task(void const * argument)
{
	 uint8_t rtcWriteData[3];
	 uint8_t rtcReadData[3];

	while(ntp_time_ok != 1){
		osDelay(50);
	}

	rtcWriteData[0] = Dec_To_BCD(sec)  & 0x7F; // seconds
	rtcWriteData[1] = Dec_To_BCD(min)  & 0x7F; // minutes
	rtcWriteData[2] = Dec_To_BCD(hour) & 0x3F; // hours

	HAL_StatusTypeDef st;

	st = HAL_I2C_Mem_Write(&hi2c1, 0x68 << 1, 0x00,
	                       I2C_MEMADD_SIZE_8BIT,
	                       rtcWriteData, 3, 100);

	if (st != HAL_OK) {
	    log_message("RTC write failed: %d\r\n", (int)st);
	}
    for(;;) {

    	if (HAL_I2C_Mem_Read(&hi2c1, 0x68 << 1, 0x00,I2C_MEMADD_SIZE_8BIT,rtcReadData, 3, 100) == HAL_OK)
		{
			uint8_t seconds = BCD_To_Dec(rtcReadData[0] & 0x7F);
			uint8_t minutes = BCD_To_Dec(rtcReadData[1] & 0x7F);
			uint8_t hours   = BCD_To_Dec(rtcReadData[2] & 0x3F);

			log_message("Time: %02u:%02u:%02u\r\n", hours, minutes, seconds);
		}
		else
		{
			log_message("RTC read failed\r\n");
		}
    	osDelay(1000);
    }
}


/* --- Communication Client/Server Task Handling --- */
void StartTCPClientDataSyncTask(void const * argument)
{
	for (;;)
	{
		log_message("\n\n ==============> SYNC DATA ROUTINE <============= \r\n");
		/*
		 * First updates my local topRMS to FRAM
		 */
		updateTableToFRAM(node_id);
		log_message("Known nodes: %d", node_count);
		// TCP client request every 60s data from all known nodes, fetch data_response and detection check + store RMS if > current top 10
		for(int i=0;i<node_count;i++){
			char ip_str[16];
			struct tcp_pcb *server_pcb;

			while (!netif_is_up(&gnetif)) {
				osDelay(100);
			}

			server_pcb = tcp_new();
			if (!server_pcb)log_message("tcp_new failed");

			err_t err = tcp_connect(server_pcb,&nodes[i].ip, LISTEN_PORT, tcp_client_connected); // callback when connected

			if(err != ERR_OK ){
				log_message("tcp_connect failed: %d", err);
				tcp_abort(server_pcb);
			}
			else{
				ipaddr_ntoa_r(&nodes[i].ip, ip_str, sizeof(ip_str));
				log_message("TCP client requested connection to %s with IP : %s \r\n",nodes[i].id, ip_str);
			}


		}
		osDelay(60000); // sync data with all nodes every 60s

	}

}

void StartPresenceBroadcastTask(void const *argument)
{
	static bool heapSizeChecked = false;
    static struct udp_pcb *pcb;
    static ip_addr_t dest_ip;
    static err_t err;

    while (!netif_is_up(&gnetif)) {
        osDelay(100);
    }

    pcb = udp_new_ip_type(IPADDR_TYPE_V4);
    if (!pcb) {
        log_message("udp_new failed\r\n");
        vTaskDelete(NULL);
    }

    err = udp_bind(pcb, IP_ADDR_ANY, 0);
    if (err != ERR_OK) {
        log_message("udp_bind failed: %d\r\n", err);
        udp_remove(pcb);
        vTaskDelete(NULL);
    }

    IP4_ADDR(ip_2_ip4(&dest_ip), 192,168,1,255);
    log_message("UDP presence socket ready");

    for(;;) {
    	char msg[128];
		char ip_str[16];
		char timestamp[32];

		ipaddr_ntoa_r(netif_ip4_addr(netif_default), ip_str, sizeof(ip_str));
		snprintf(timestamp, sizeof(timestamp), "2000-00-0000:00:00Z");

        int len = snprintf(msg, sizeof(msg),
                           "{"
                             "\"type\":\"presence\","
                             "\"id\":\"%s\","
                             "\"ip\":\"%s\","
                             "\"timestamp\":\"%s\""
                           "}",
                           node_id,
						   ip_str,
						   timestamp);

        struct pbuf *pb = pbuf_alloc(PBUF_TRANSPORT, len, PBUF_RAM);
        memcpy(pb->payload, msg, len);

        err = udp_sendto(pcb, pb, &dest_ip, LISTEN_PORT);
        pbuf_free(pb);

        log_message("Presence message sent (err=%d)", err);
        if(!heapSizeChecked){
			UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(acquisitionTaskHandle);
			log_message("PresenceBroadcastTask min free : %lu words\r\n",uxHighWaterMark);
			heapSizeChecked = true;
		}
        osDelay(10000); // Every 10s => sync data every 60 s, thus nucleo should have the remote nucleo's info locally as NodeInfo struct before handling data_response
    }
}




void StartUDPServerTask(void const * argument)
{
	static struct udp_pcb *server_pcb;
	static err_t err;

	while (!netif_is_up(&gnetif)) {
		osDelay(100);
	}

	server_pcb = udp_new_ip_type(IPADDR_TYPE_V4);
	if (!server_pcb) {
		log_message("udp_new failed");
		vTaskDelete(NULL);
	}

	err = udp_bind(server_pcb, IP_ADDR_ANY, LISTEN_PORT);
	if (err != ERR_OK) {
		log_message("udp_bind failed: %d", err);
		udp_remove(server_pcb);
		vTaskDelete(NULL);
	}

	udp_recv(server_pcb, udp_receive_callback, NULL); // Callback function

	log_message("UDP server listening on port 12345");

    for(;;) {
    	osDelay(1);
    }
}

void StartTCPServerTask(void const * argument)
{
	struct tcp_pcb *listen_pcb;

	while (!netif_is_up(&gnetif)) {
		osDelay(100);
	}

	listen_pcb = tcp_new();
	if (!listen_pcb)log_message("tcp_new failed");

	err_t err = tcp_bind(listen_pcb, IP_ADDR_ANY, LISTEN_PORT);
	if (err != ERR_OK) {
		log_message("tcp_bind failed: %d", err);
		tcp_close(listen_pcb);
	}

	listen_pcb = tcp_listen(listen_pcb);
	if (!listen_pcb) {
		log_message("tcp_listen failed");
	}

	tcp_accept(listen_pcb, tcp_accept_cb);

	log_message("TCP server listening on port 12345");

	for(;;) {
		osDelay(1);
	}
}



/* ============================================================================== */

/* ======================= USER CALLBACK FUNCTION DEFINITIONS ======================= */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == USER_Btn_Pin)buttonPressed = !buttonPressed;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	flagConversion = 1;
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for LWIP */
  MX_LWIP_Init();
  /* USER CODE BEGIN 5 */


  /* Wait until the interface is up */
  while (!netif_is_up(&gnetif)) {
      osDelay(100);
  }

  /* Print IP configuration */
  char ip_buf[16];
  char mask_buf[16];
  char gw_buf[16];

  /* Convert values */
  ip4addr_ntoa_r(netif_ip4_addr(&gnetif), ip_buf, sizeof(ip_buf));
  ip4addr_ntoa_r(netif_ip4_netmask(&gnetif), mask_buf, sizeof(mask_buf));
  ip4addr_ntoa_r(netif_ip4_gw(&gnetif), gw_buf, sizeof(gw_buf));

  /* Print using your logger or printf */
  log_message("IP     : %s", ip_buf);
  log_message("MASK   : %s", mask_buf);
  log_message("GW     : %s", gw_buf);

  /* Print MAC address */
  log_message("MAC    : %02X:%02X:%02X:%02X:%02X:%02X",
      gnetif.hwaddr[0], gnetif.hwaddr[1], gnetif.hwaddr[2],
      gnetif.hwaddr[3], gnetif.hwaddr[4], gnetif.hwaddr[5]);

  ip_addr_t dns;
  ipaddr_aton("192.168.1.1", &dns);
  dns_setserver(0, &dns);

  const ip_addr_t* s0 = dns_getserver(0);
  log_message("DNS (Gateway ip) = %s\n", ipaddr_ntoa(s0));


  char buf2[64];
  snprintf(buf2, sizeof(buf2), "Free heap after MX_LWIP_Init: %u bytes",
           (unsigned)xPortGetFreeHeapSize());
  log_message("%s", buf2);

  log_message("Initializing my node as NodeInfo struct and storing in nodes array \r\n");
  NodeInfo new_node;

  ipaddr_aton(ip_buf, &new_node.ip); // Convert str to ipaddress
  strncpy(new_node.id, node_id, sizeof(new_node.id));

  nodes[node_count] = new_node;   // nodes[0]
  node_count++;
  log_message("Node count is %d",node_count);
  print_all_nodes();
//  osDelay(50);
  restoringDataFromFRAMForAliveNode(node_id);

  /* Main loop */
  for(;;) {
      osDelay(1);
  }

  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
