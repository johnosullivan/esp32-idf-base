#include <lwip/sockets.h>
#include <string.h>
#include <stdint.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include <esp_system.h>
#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_log.h>
#include <esp_err.h>
#include <nvs_flash.h>
#include <lwip/err.h>
#include <lwip/sockets.h>
#include <lwip/sys.h>
#include <lwip/netdb.h>
#include <lwip/dns.h>
#include <byteswap.h>

#include "wifi_manager.h"
#include "dns_server.h"

static const char TAG[] = "mihome_esp32_dns_server";
static TaskHandle_t task_dns_server = NULL;
int socket_fd;

void dns_server_start() {
    xTaskCreate(&dns_server, "dns_server", 3072, NULL, WIFI_MANAGER_TASK_PRIORITY-1, &task_dns_server);
}

void dns_server_stop(){
	if(task_dns_server){
		vTaskDelete(task_dns_server);
		close(socket_fd);
		task_dns_server = NULL;
	}

}

void dns_server(void *pvParameters) {
    struct sockaddr_in ra;

    /* Set redirection DNS hijack to the access point IP */
    ip4_addr_t ip_resolved;
    inet_pton(AF_INET, DEFAULT_AP_IP, &ip_resolved);

    /* Create UDP socket */
    socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd < 0){
        ESP_LOGE(TAG, "Failed to create socket");
        exit(0);
    }

    /* Bind to port 53 (typical DNS Server port) */
    esp_netif_ip_info_t ip;
    esp_netif_t* netif_sta = wifi_manager_get_esp_netif_sta();
    ESP_ERROR_CHECK(esp_netif_get_ip_info(netif_sta, &ip));
    ra.sin_family = AF_INET;
    ra.sin_addr.s_addr = ip.ip.addr;
    ra.sin_port = htons(53);
    if (bind(socket_fd, (struct sockaddr *)&ra, sizeof(struct sockaddr_in)) == -1) {
        ESP_LOGE(TAG, "Failed to bind to 53/udp");
        close(socket_fd);
        exit(1);
    }

    struct sockaddr_in client;
    socklen_t client_len;
    client_len = sizeof(client);
    int length;
    uint8_t data[DNS_QUERY_MAX_SIZE];	/* dns query buffer */
    uint8_t response[DNS_ANSWER_MAX_SIZE]; /* dns response buffer */
    char ip_address[INET_ADDRSTRLEN]; /* buffer to store IPs as text. This is only used for debug and serves no other purpose */
    char *domain; /* This is only used for debug and serves no other purpose */
    int err;

    ESP_LOGI(TAG, "DNS Server listening on 53/udp");

    /* Start loop to process DNS requests */
    for(;;) {

    	memset(data, 0x00,  sizeof(data)); /* reset buffer */
        length = recvfrom(socket_fd, data, sizeof(data), 0, (struct sockaddr *)&client, &client_len); /* read udp request */

        /*if the query is bigger than the buffer size we simply ignore it. This case should only happen in case of multiple
         * queries within the same DNS packet and is not supported by this simple DNS hijack. */
        if ( length > 0   &&  ((length + sizeof(dns_answer_t)-1) < DNS_ANSWER_MAX_SIZE)   ) {

        	data[length] = '\0'; /*in case there's a bogus domain name that isn't null terminated */

            /* Generate header message */
            memcpy(response, data, sizeof(dns_header_t));
            dns_header_t *dns_header = (dns_header_t*)response;
            dns_header->QR = 1; /*response bit */
            dns_header->OPCode  = DNS_OPCODE_QUERY; /* no support for other type of response */
            dns_header->AA = 1; /*authoritative answer */
            dns_header->RCode = DNS_REPLY_CODE_NO_ERROR; /* no error */
            dns_header->TC = 0; /*no truncation */
            dns_header->RD = 0; /*no recursion */
            dns_header->ANCount = dns_header->QDCount; /* set answer count = question count -- duhh! */
            dns_header->NSCount = 0x0000; /* name server resource records = 0 */
            dns_header->ARCount = 0x0000; /* resource records = 0 */

            /* copy the rest of the query in the response */
            memcpy(response + sizeof(dns_header_t), data + sizeof(dns_header_t), length - sizeof(dns_header_t));

            /* extract domain name and request IP for debug */
            inet_ntop(AF_INET, &(client.sin_addr), ip_address, INET_ADDRSTRLEN);
            domain = (char*) &data[sizeof(dns_header_t) + 1];
            for(char* c=domain; *c != '\0'; c++){
            	if(*c < ' ' || *c > 'z') *c = '.'; /* technically we should test if the first two bits are 00 (e.g. if( (*c & 0xC0) == 0x00) *c = '.') but this makes the code a lot more readable */
            }
            ESP_LOGD(TAG, "Replying to DNS request for %s from %s", domain, ip_address);

            /* create DNS answer at the end of the query*/
            dns_answer_t *dns_answer = (dns_answer_t*)&response[length];
            dns_answer->NAME = __bswap_16(0xC00C); /* This is a pointer to the beginning of the question. As per DNS standard, first two bits must be set to 11 for some odd reason hence 0xC0 */
            dns_answer->TYPE = __bswap_16(DNS_ANSWER_TYPE_A);
            dns_answer->CLASS = __bswap_16(DNS_ANSWER_CLASS_IN);
            dns_answer->TTL = (uint32_t)0x00000000; /* no caching. Avoids DNS poisoning since this is a DNS hijack */
            dns_answer->RDLENGTH = __bswap_16(0x0004); /* 4 byte => size of an ipv4 address */
            dns_answer->RDATA = ip_resolved.addr;

            err = sendto(socket_fd, response, length+sizeof(dns_answer_t), 0, (struct sockaddr *)&client, client_len);
            if (err < 0) {
            	ESP_LOGE(TAG, "UDP sendto failed: %d", err);
            }
        }

        taskYIELD(); /* allows the freeRTOS scheduler to take over if needed. DNS daemon should not be taxing on the system */

    }
    close(socket_fd);

    vTaskDelete ( NULL );
}