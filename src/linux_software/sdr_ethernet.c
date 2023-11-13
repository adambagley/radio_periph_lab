#include <stdio.h>
#include <sys/mman.h> 
#include <fcntl.h> 
#include <unistd.h>
#include <time.h>
#include <netdb.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include <pthread.h>
#define _BSD_SOURCE

#define RADIO_TUNER_FAKE_ADC_PINC_OFFSET 0
#define RADIO_TUNER_TUNER_PINC_OFFSET 1
// the write of this register is used for resetting the DDS 
#define RADIO_TUNER_CONTROL_REG_OFFSET 2
// the read of this same register is used for getting how many readable words are in the FIFO
#define RADIO_TUNER_FIFO_RD_CNT_REG_OFFSET 2
// the read of this register is used for getting the FIFO read data
#define RADIO_TUNER_FIFO_RD_DATA_REG_OFFSET 3
#define RADIO_PERIPH_ADDRESS 0x43c00000

#define UDP_PAYLOAD_LEN 1026
#define NUM_SAMPLES_PER_PACKET 256
const char *restrict udp_port = "25344";
int udp_client_socket;
volatile unsigned int *my_periph;    
//char *restrict udp_dest_ip = "192.168.1.3";
char udp_dest_ip[15] = "192.168.1.3";
uint8_t ethernet_enabled = 0; 
uint32_t tune_frequency = 30e6; // start with 30MHz
uint32_t adc_frequency = 30e6+1e3; // start with 30MHz + 1kHz

// the body of this function is from UDP client example in Linux man for 'getaddrinfo'
int open_udp_client_socket(char* host) {
    int              sfd, s;
    struct addrinfo  hints;
    struct addrinfo  *result, *rp;

    /* Obtain address(es) matching host. */

    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_UNSPEC;    /* Allow IPv4 or IPv6 */
    hints.ai_socktype = SOCK_DGRAM; /* Datagram socket */
    hints.ai_flags = 0;
    hints.ai_protocol = 0;          /* Any protocol */

    s = getaddrinfo(host, udp_port, &hints, &result);
    if (s != 0) {
        fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(s));
        exit(EXIT_FAILURE);
    }

    /* getaddrinfo() returns a list of address structures.
       Try each address until we successfully connect(2).
       If socket(2) (or connect(2)) fails, we (close the socket
       and) try the next address. */

    for (rp = result; rp != NULL; rp = rp->ai_next) {
        sfd = socket(rp->ai_family, rp->ai_socktype,
                     rp->ai_protocol);
        if (sfd == -1)
            continue;

        if (connect(sfd, rp->ai_addr, rp->ai_addrlen) != -1)
            break;                  /* Success */

        close(sfd);
    }

    freeaddrinfo(result);           /* No longer needed */

    if (rp == NULL) {               /* No address succeeded */
        fprintf(stderr, "Could not connect\n");
        exit(EXIT_FAILURE);
    }
    return sfd;
}

void *ethernet_thread(void *vargp) {

    // open UDP client socket with destination of user-specified host
    udp_client_socket = open_udp_client_socket(udp_dest_ip);

    // send the UDP packets with payloads containing data read from FPGA FIFO
    // which is getting written to by FPGA radio peripheral
    uint8_t payload [UDP_PAYLOAD_LEN]; 
    uint8_t *payload_byte_ptr;
    uint32_t *payload_word_ptr;
    uint32_t payload_samples [NUM_SAMPLES_PER_PACKET];
    uint32_t fifo_rd_data_cnt;
    uint32_t num_words_read_from_fifo;
    uint16_t counter = 0;
    // send packets indefinitely for now.
    while (1) {
        if (ethernet_enabled) {
            // insert counter value as a 2byte little-endian 'sequence number'
            payload[0] = counter & 0xff;
            payload[1] = (counter >> 8) & 0xff;
            payload_byte_ptr = &payload[2];
            payload_word_ptr = (uint32_t*)payload_byte_ptr;
            num_words_read_from_fifo = 0;
            while (num_words_read_from_fifo<NUM_SAMPLES_PER_PACKET) {
                fifo_rd_data_cnt = *(my_periph+RADIO_TUNER_FIFO_RD_CNT_REG_OFFSET);
                for (int i=0; i<fifo_rd_data_cnt && num_words_read_from_fifo<NUM_SAMPLES_PER_PACKET; ++i, ++num_words_read_from_fifo, ++payload_word_ptr)
                    *payload_word_ptr = *(my_periph+RADIO_TUNER_FIFO_RD_DATA_REG_OFFSET);
            }
        
            while (1)
                // keep trying to send the packet until successful
                if (!write(udp_client_socket, payload, UDP_PAYLOAD_LEN) != UDP_PAYLOAD_LEN)
                    break;
    
            ++counter;
        }
    }
    exit(EXIT_SUCCESS);
}

// the below code uses a device called /dev/mem to get a pointer to a physical
// address.  We will use this pointer to read/write the custom peripheral
volatile unsigned int * get_a_pointer(unsigned int phys_addr) {
    int mem_fd = open("/dev/mem", O_RDWR | O_SYNC); 
    void *map_base = mmap(0, 4096, PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, phys_addr); 
    volatile unsigned int *radio_base = (volatile unsigned int *)map_base; 
    return (radio_base);
}


void radioTuner_tuneRadio(volatile unsigned int *ptrToRadio, float tune_frequency) {
    float pinc = (-1.0*tune_frequency)*(float)(1<<27)/125.0e6;
    *(ptrToRadio+RADIO_TUNER_TUNER_PINC_OFFSET)=(int)pinc;
    printf("tune_frequency is currently %.0f Hz\n\n",tune_frequency);
}

void radioTuner_setAdcFreq(volatile unsigned int* ptrToRadio, float freq) {
    float pinc = freq*(float)(1<<27)/125.0e6;
    *(ptrToRadio+RADIO_TUNER_FAKE_ADC_PINC_OFFSET) = (int)pinc;
    printf("adc_frequency is currently %.0f Hz\n\n",freq);
}

void print_help(){
    printf("\n");
    printf("tune_frequency is currently %lu Hz\n",tune_frequency);
    printf("adc_frequency is currently %lu Hz\n",adc_frequency);
    if (ethernet_enabled)
        printf("Streaming of radio output data to ethernet is currently enabled\n");
    else
        printf("Streaming of radio output data to ethernet is currently disabled\n");
    printf("Destination IP address of UDP packets is currently %s\n\n",udp_dest_ip);

    printf("Enter 't' followed by <frequency> followed by [enter] to set the tune frequency.\n");
    printf("Example: t 30000000\n");
    printf("Enter 'f' followed by <frequency> followed by [enter] to set the ADC frequency.\n");
    printf("Example: f 30001000\n");
    printf("Enter 'u' followed by [enter] to increment ADC frequency by 100Hz.\n");
    printf("Enter 'U' followed by [enter] to increment ADC frequency by 1000Hz.\n");
    printf("Enter 'd' followed by [enter] to decrement ADC frequency by 100Hz.\n");
    printf("Enter 'D' followed by [enter] to decrement ADC frequency by 1000Hz.\n");
    printf("Enter 'z' followed by [enter] to reset DDS phase to 0.\n");
    printf("Enter 'i' followed by <ip address> in the format 'X.X.X.X' followed by [enter] to set the UDP destination IP address.\n");
    printf("Example: i 192.168.1.3\n");
    printf("Enter 'e' followed by [enter] to toggle enabling of streaming of radio output data to ethernet.\n");
    printf("Enter 'h' followed by [enter] to show this help menu again.\n\n");
}

int main() {
    // first, get a pointer to the peripheral base address using /dev/mem and the function mmap
    my_periph = get_a_pointer(RADIO_PERIPH_ADDRESS);    

    printf("\r\n\r\n\r\nLinux SDR With Ethernet: Adam Bagley\n\r");

    printf("\nResetting DDS\n\n");
    *(my_periph+RADIO_TUNER_CONTROL_REG_OFFSET) = 1;
    *(my_periph+RADIO_TUNER_CONTROL_REG_OFFSET) = 0;

    radioTuner_tuneRadio(my_periph,tune_frequency);
    radioTuner_setAdcFreq(my_periph,adc_frequency);
    
    // kick off separate thread for sending UDP packets
    pthread_t thread_id;
    pthread_create(&thread_id, NULL, ethernet_thread, NULL);

    // user CLI
    print_help();
    char cmd;
    while (scanf(" %c",&cmd)) {
        if (cmd=='z') {
            printf("Resetting DDS\n\n");
            *(my_periph+RADIO_TUNER_CONTROL_REG_OFFSET) = 1;
            *(my_periph+RADIO_TUNER_CONTROL_REG_OFFSET) = 0;
        } else if (cmd=='e') {
            if (!ethernet_enabled) {
                ethernet_enabled = 1;
                printf("Streaming of radio output data to ethernet is now enabled\n\n");
            } else {
                ethernet_enabled = 0;
                printf("Streaming of radio output data to ethernet is now disabled\n\n");
            }
        } else if (cmd=='i') {
            scanf(" %s",&udp_dest_ip);
            udp_client_socket = open_udp_client_socket(udp_dest_ip);
            printf("Destination IP address of UDP packets is now %s\n\n",udp_dest_ip);
        } else if (cmd=='t') {
            char freq_str[10] = "0000000000";
            scanf(" %s",&freq_str);
            tune_frequency = atoi(freq_str);
            radioTuner_tuneRadio(my_periph,tune_frequency);
        }
        else if (cmd=='f') {
            char freq_str[10] = "0000000000";
            scanf(" %s",&freq_str);
            adc_frequency = atoi(freq_str);
            radioTuner_setAdcFreq(my_periph,adc_frequency);
        }
        else if (cmd=='u') {
            adc_frequency+=100;
            radioTuner_setAdcFreq(my_periph,adc_frequency);
        }
        else if (cmd=='U') {
            adc_frequency+=1000;
            radioTuner_setAdcFreq(my_periph,adc_frequency);
        }
        else if (cmd=='d') {
            adc_frequency-=100;
            radioTuner_setAdcFreq(my_periph,adc_frequency);
        }
        else if (cmd=='D') {
            adc_frequency-=1000;
            radioTuner_setAdcFreq(my_periph,adc_frequency);
        }
        else if (cmd=='h')
            print_help();
        else {
            printf("Unknown command: %c\n\n",cmd);
            print_help();
        }
    }

    return 0;
}
