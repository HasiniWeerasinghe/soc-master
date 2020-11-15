/*
   Project - Coordinated Multi Device Controller System Using Single Multipair Cable [FYP]
   Modules - ARM Processor Source Code
   Author  - Hasini Weerasinghe

   Description - This the corresponding source code to ...
*/
/// C library headers
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <string.h>
#include <sys/types.h>
#include <dirent.h>


/// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

/// pthread
#include <pthread.h>
#include <semaphore.h>

/// Header Files
//#include <decoder.h>
//#include <rx_thread.h>
//#include <tx_thread.h>
//#include <pulseCount.h>

/// Configuration Settings

/// No of lines in config.txt File
#define no_of_lines_config_txt 4
///
#define motion_initial_data_length 12
/// Decoder Fifo
#define DECODER_FIFO_SIZE 5
#define DECODER_FIFO_MAX_STR_LEN 256
/// Fifo Array
/// RX
#define FIFO_ARRAY_SIZE 16
#define FIFO_ARRAY_MAX_STR_LEN 12
/// TX
#define TX_FIFO_DECODER_SIZE 16
#define TX_FIFO_DECODER_MAX_STR_LEN 12// this should be always greater than 11

///for tx thread
#define MAX_STR_LEN 12
//#define FPGA_READ_FIFO_SIZE
//#define FPGA_READ_FIFO_MAX_STR_LEN 12/
//char* first_element_of_received_data;
//short* len_of_received_data;
//extern writerSem;
//extern readerSem;

sem_t writerSem;//decoder-rx fifo
sem_t readerSem;//decoder-rx fifo
//sem_t writerSem_fifoarray;//fpgawrite-Decoder fifo
//sem_t readerSem_fifoarray;//fpga write-decoder fifo
//sem_t writerSem_tx;//tx thread-Decoder fifo
//sem_t readerSem_tx;//tx thread-Decoder fifo
//

struct xmlnames{
    unsigned char b[10];
};



unsigned char processed_data_fifo_insert [FIFO_ARRAY_MAX_STR_LEN];
unsigned  char *master_pointer;
char fifo_array[FIFO_ARRAY_SIZE][FIFO_ARRAY_MAX_STR_LEN];
unsigned char read_pointer;
unsigned char write_pointer;


unsigned char motion_initial_data[motion_initial_data_length]; // twice the required total bytes to represent the motion details.
//char *motion_initial_data_pass;//make a copy of data
unsigned char *motion_init_array_start_pointer[255]; // [0]- starting pointer, [1]-no of bytes for motion
short motion_init_array_byte_count[255];
char thread_status[255]={0}; // 00000000-not exist a threadexist(should be update by motion thread), 1111xxxx-new data set available(should be update by decoder), xxxx1111-thread exist(should be update by motion thread)

pthread_mutex_t lock;
//pthread_mutex_t lock_1;
pthread_mutex_t write_lock;//FPGA write -decoder
pthread_mutex_t read_lock;//FPGA write -decoder
pthread_mutex_t write_lock_tx;//tx thread and other thread
pthread_mutex_t read_lock_tx;//tx thread and other thread

pthread_cond_t data_recieve;
pthread_cond_t master_control;
pthread_cond_t fifo_read;//FPGA write -decoder
pthread_cond_t fifo_write;//FPGA write -decoder
pthread_cond_t fifo_read_tx;//tx thread-other thread
pthread_cond_t fifo_write_tx;//tx thread-other thread

pthread_t motion_threads[255];
char sendBuff[MAX_STR_LEN];//for tx thread

struct args{//pass arguments between rx thread and decoder.
    unsigned char *pointer[DECODER_FIFO_SIZE];
    unsigned char *length[DECODER_FIFO_SIZE];
    bool *status_array[DECODER_FIFO_SIZE];
};

/// RX_AND_DECODER_FIFO
unsigned char fifo_array_rx[DECODER_FIFO_SIZE][DECODER_FIFO_MAX_STR_LEN];
unsigned char length_rx[DECODER_FIFO_SIZE];
//bool status_array_rx[DECODER_FIFO_SIZE]={0};

struct args decoder_arg;//={&fifo_array_rx[0][0],&length_rx[0],&status_array_rx[0]};

/// TX_FIFOs
unsigned char fifo_array_tx[TX_FIFO_DECODER_SIZE][TX_FIFO_DECODER_MAX_STR_LEN];
unsigned char fifo_array_tx_writepointer=0;
unsigned char fifo_array_tx_readpointer=0;

/// FROM_FPGA_READ_TO_TX_FIFOs
/*unsigned char fifo_array_FPGA_READ[FPGA_READ_FIFO_SIZE][FPGA_READ_FIFO_MAX_STR_LEN];
unsigned char fifo_array_FPGA_READ_writepointer=0;
unsigned char fifo_array_FPGA_READ_readpointer=0;*/

char delete_success_header = 0b01000110;
char delete_unsuccess_header = 0b01001110;
char set_success_header = 0b01000101;
char set_unsuccess_header = 0b01001101;
char Successful_data_recieve_header = 0b11100010;
char transmit_array[TX_FIFO_DECODER_MAX_STR_LEN];


//struct args_tx tx_arg;//={&fifo_array_rx[0][0],&length_rx[0],&status_array_rx[0]};
struct dirent *pp;//this is for xml read available


void *readThread(void *parameters)
{
//char readBuff[DECODER_FIFO_MAX_STR_LEN];

 int fd;

 fd = *((int*)parameters);

 while(1)
 {
 for(unsigned char i=0;i<DECODER_FIFO_SIZE;i++){
    sem_wait(&writerSem);
    ssize_t len;// ?signed size_t?. ssize_t is able to represent the number -1, which is returned by several system calls and library functions as a way to indicate error.

    len = read(fd, &fifo_array_rx[i], DECODER_FIFO_MAX_STR_LEN);
    if(len==-1){
        printf("error in reading");

    }else{
        //status_array_rx[i]=~status_array_rx[i];
        length_rx[i]=len;


    }
    sem_post(&readerSem);
    //printf("Read %li bytes\n", len);
    //pthread_cond_signal(&data_recieve);

 }

//
//
}//while
}

void *writeThread(void *parameters)
{

//sem_post(&readerSem_tx);
//
//memset(&sendBuff[0], 0, MAX_STR_LEN);//copy zero to all in sendbuffer

memcpy(&sendBuff[0],&fifo_array_tx[fifo_array_tx_readpointer],TX_FIFO_DECODER_MAX_STR_LEN);
fifo_array_tx_readpointer++;
//
int fd;
//
fd = *((int*)parameters);
//
while(1)
{
pthread_mutex_lock(&read_lock);
	//sem_post(&readerSem_fifoarray);
if(fifo_array_tx_readpointer==fifo_array_tx_writepointer){
        /// FIFO is empty
    pthread_cond_wait(&fifo_write_tx,&read_lock_tx);//wait till data is read by reading thread
}
 write(fd, &sendBuff[0], strlen(&sendBuff[0]) );
//
//  // sleep enough to transmit the length plus receive 25:
//  // approx 100 uS per char transmit
usleep((strlen(&sendBuff[0]) + 25) *9);
pthread_cond_signal(&fifo_read_tx);//wake up fpga read thread
pthread_mutex_unlock(&read_lock_tx);
//sem_wait(&writerSem_tx);
//usleep(500*1000);
}/*while*/
//
// //pthread_exit(0);
}/*readThread*/




//motion_initial_data_pass=malloc(2*1);


void *decoder(void *input){
    /// ----------------------------
    char xml_name[10];
    unsigned char filename[14]; /// File Name + ".txt"
    FILE *fptr;
    char line[no_of_lines_config_txt][40]={"ACTIVE_XML 1234567890\n"}; // maximum line width 40, maximum no of lines 4
    /// ----------------------------
	char xmlerror;//this is a control command
    struct args *argument = input;

    unsigned char received_array_len=0;
    unsigned char current_index=0; /// index number of processing byte of received array
    unsigned char heading=0;
    unsigned char sec_heading=0;
    unsigned char local_len_count;

    bool xml_name_received;
    unsigned char xml_file_name[10];
    bool read_write=0;


    unsigned char ch;
    unsigned char count =0; // to count the processed byte count of the sec_heading
    unsigned char l=0; // to save the no of bytes for each heading 2 cmds
    unsigned char arr_count=0;

    unsigned char slave_addr;
    unsigned char reg_addr[2];
    unsigned char data[4];
    bool slve_addr_recieved=0;
    bool reg_addr_recieved=0;
    bool data_received=0;
    unsigned char addr_count=0;
    unsigned char data_count=0;

    unsigned char pin_no;
    unsigned char time_trigger_data[8];
    bool pin_no_recieved=0;

    bool motion_num_recieved=0;
    bool timing_recieved=0;
    bool slave_addr_recieved=0;
    bool angle_recieved=0;
    bool all_data_recieved=0;


    bool status_of_the_current_process=1; /// Memorize the status of any given process [1- Success,  0- Unsuccess]
//    unsigned char digital_data;
//    unsigned char *start=NULL;
//    unsigned char *end=NULL;

	char boot_file_name[15] = "boot_config.txt";

    short count_1=0;// motion_init_array_byte_count
    unsigned char *next_fill_loction=&motion_initial_data[0];
    char motion_no;

    bool local_status_array[DECODER_FIFO_SIZE]={0};
    unsigned char fifo_read_pointer=0;
    unsigned char arr_copy[DECODER_FIFO_MAX_STR_LEN];

/// OUTER LOOP
    while(1)
    {

/// LOAD DATA FROM DECODER FIFO
    if(~(received_array_len)){
        while(1){
            sem_post(&readerSem);
            if(local_status_array[fifo_read_pointer]!= *argument->status_array[fifo_read_pointer]){
                local_status_array[fifo_read_pointer]=~local_status_array[fifo_read_pointer];
                memcpy(arr_copy,argument->pointer[fifo_read_pointer],*argument->length[fifo_read_pointer]); /// copy data to the local array from fifo
                current_index=0;
                sem_wait(&writerSem);
                received_array_len=*(argument->length[fifo_read_pointer]);
                fifo_read_pointer++;
            }else{
                sem_wait(&writerSem);
                usleep(1000);
            }
        }
    }

    received_array_len=received_array_len-1; /// reduce the length
    switch(heading){ /// Read frame Heading
        case 0b00000000: /// default case
            printf("frame begining\n");
            heading=arr_copy[current_index]>>4;
            if(heading==0b00000001){
                processed_data_fifo_insert[0]=0b00000001;
            }else if(heading==0b00000010){
                processed_data_fifo_insert[0]=0b00000011;
            }else if(heading==0b00000011){//master control
                master_pointer=&arr_copy[current_index]; /// - need to debug
                pthread_mutex_lock(&lock);
                pthread_cond_signal(&master_control);//wake up fpga read thread
                pthread_mutex_unlock(&lock);
            }
            count++;
            goto while_end;
        case 0b00000001://parallel

            switch(sec_heading){
                label:
                case 0b00000000:
                     sec_heading=arr_copy[current_index]>>4;
                     if(sec_heading!=0b00001011 ||sec_heading==0b00001100){
                        processed_data_fifo_insert[count]=arr_copy[current_index];
                        count++;
                     }
                     switch(sec_heading)
                        case 0b00000001://reg write local memory
                            l=9;
                            goto while_end;
                        case 0b00000010://reg read local memory
                             l=9;
                            goto while_end;
                        case 0b00001001://reg write user memory
                             l=9;
                            goto while_end;
                        case 0b00001010://reg read user memory
                             l=9;
                            goto while_end;
                        case 0b00000011://read triggered time
                             l=12;
                            goto while_end;
                        case 0b00000100://digital input read
                             l=5;
                            goto while_end;
                        case 0b00000101://digital input write
                             l=5;
                            goto while_end;
                        case 0b00000110://analog
                             l=6;
                            goto while_end;
                        case 0b00000111://PWM
                             l=6;
                            goto while_end;
                        case 0b00001000://I2C
                             l=10;
                            goto while_end;


                case 0b00001011://motion
                    if(motion_num_recieved){

                        //count_1=0;
                         while(arr_copy[current_index]!=0b00000000){
                            count_1++;
                            next_fill_loction=&arr_copy[current_index]; // received byte by byte after the heading byte until end motion(except motion heading byte, motion no byte, end byte)
                            if(next_fill_loction==&motion_initial_data[motion_initial_data_length-1]){

                                next_fill_loction=&motion_initial_data[0];
                            }else{
                                next_fill_loction++;
                            }
                            goto while_end;
                        }

                        motion_init_array_byte_count[motion_no-1]=count_1;
                        count_1=0;
                        if(thread_status[motion_no-1]==0){

                                    //create thread;
                                    /*pthread_create(motion_threads[motion_no-1],NULL,pulse_count,(void*)&motion_no);
                                    thread_status[motion_no-1]=0b1111111;
                                    motion_num_recieved=0;*/

                        }else{
                            thread_status[motion_no-1]=0b1111111;// update the thread_status array
                            }

                    }else{
                        motion_no=arr_copy[current_index];
                        motion_num_recieved=1;
                        motion_init_array_start_pointer[motion_no-1]=next_fill_loction;
                    }
                    goto while_end;

                default:
                    if(l==count){
                        if(arr_copy[current_index]>>4==12){/// all data had received and release the heading and secaa_heading
                            processed_data_fifo_insert[0]=0b00000010;
                            pthread_mutex_lock(&write_lock);
							//sem_wait(&writerSem_fifoarray);
                            if((write_pointer==15 && read_pointer==0)|| (read_pointer-write_pointer)==1){
                                /// FIFO is full
                                pthread_cond_wait(&fifo_read,&write_lock);//wait till data is read by reading thread
                            }
                            memcpy(&fifo_array[write_pointer],processed_data_fifo_insert,FIFO_ARRAY_MAX_STR_LEN); /// pass data
                            write_pointer++;
                            memcpy(&fifo_array[write_pointer],&Successful_data_recieve_header,FIFO_ARRAY_MAX_STR_LEN); /// pass successful data receive header
                            write_pointer++;

                            pthread_cond_signal(&fifo_write);//wake up fpga read thread
                            pthread_mutex_unlock(&write_lock);
							//sem_post(&readerSem_fifoarray);

                            sec_heading=0;
                            heading=0;
                            count=0;
                            //pass pointer to write thread
                        }else{
                            pthread_mutex_lock(&write_lock);
							//sem_wait(&writerSem_fifoarray);

                            if((write_pointer==15 && read_pointer==0)|| (read_pointer-write_pointer)==1){
                                /// FIFO is full
                                pthread_cond_wait(&fifo_read,&write_lock);//wait till data is read by reading thread
                            }
                            memcpy(&fifo_array[write_pointer],processed_data_fifo_insert,FIFO_ARRAY_MAX_STR_LEN); /// fifo_array[write_pointer]= pointer;//required data;
                            write_pointer++;
                            pthread_cond_signal(&fifo_write);//wake up fpga read thread
                            pthread_mutex_unlock(&write_lock);
							//sem_post(&readerSem_fifoarray);

                            processed_data_fifo_insert[0]=0b00000001;
                            sec_heading=0;
                            count=1;
                            goto label;

                        }
                    }else{
                        processed_data_fifo_insert[count]=arr_copy[current_index];
                        count++;
                    }
                    goto while_end;
           }
        case 0b00000010:
            printf("inside sequential\n");
            switch(sec_heading){
                case 0b00000000:
                     sec_heading=arr_copy[current_index]>>4;
                     if(sec_heading!=0b00001011 ||sec_heading==0b00001100){
                        processed_data_fifo_insert[count]=arr_copy[current_index];
                        count++;
                     }
                     switch(sec_heading)
                        case 0b00000001://reg write local memory
                            l=9;
                            goto while_end;
                        case 0b00000010://reg read local memory
                             l=9;
                            goto while_end;
                        case 0b00001001://reg write user memory
                             l=9;
                            goto while_end;
                        case 0b00001010://reg read user memory
                             l=9;
                            goto while_end;
                        case 0b00000011://read triggered time
                             l=12;
                            goto while_end;
                        case 0b00000100://digital input read
                             l=5;
                            goto while_end;
                        case 0b00000101://digital input write
                             l=5;
                            goto while_end;
                        case 0b00000110://analog
                             l=6;
                            goto while_end;
                        case 0b00000111://PWM
                             l=6;
                            goto while_end;
                        case 0b00001000://I2C
                             l=10;
                            goto while_end;


//                case 0b00001011://motion
//                 if(motion_num_recieved){
//                        motion_init_array_start_pointer[motion_no-1]=next_fill_loction;
//                        //count_1=0;
//                         while(count_1<=5){
//                            count_1++;
//                            &next_fill_loction=arr_copy[current_index]; // received byte by byte after the heading byte until end motion(except motion heading byte, motion no byte, end byte)
////                            if(next_fill_loction==&motion_initial_data[motion_initial_data_length-1]){
////                                next_fill_loction=&motion_initial_data[0];
////                            }else{
////                                next_fill_loction++;
////                            }
//                            goto while_end;
//                        }
//
//                        motion_init_array_byte_count[motion_no-1]=count_1;
//                        count_1=0;
//                        if(thread_status[motion_no-1]==0)
//
//                                    //create thread;
//                                    pthread_create(motion_threads[motion_no-1],NULL,pulse_count,(void*)&motion_no);
//                                    thread_status[motion_no-1]=0b1111111;
//
//                        else
//                            thread_status[motion_no-1]=0b1111111;// update the thread_status array
//                        motion_num_recieved=0;
//                    }else{
//                        motion_no=arr_copy[current_index];
//                        motion_num_recieved=1;
//
//
//                    }
//
//                    goto while_end;

                default:
                    if(l==count){
                        // all data had received and release the heading and secaa_heading
                            pthread_mutex_lock(&write_lock);
							//sem_wait(&writerSem_fifoarray);
                            if((write_pointer==15 && read_pointer==0)|| (read_pointer-write_pointer)==1){
                                /// FIFO is full
                                pthread_cond_wait(&fifo_read,&write_lock);//wait till data is read by reading thread
                            }
                            memcpy(&fifo_array[write_pointer],processed_data_fifo_insert,FIFO_ARRAY_MAX_STR_LEN); /// fifo_array[write_pointer]= pointer;//required data;fifo_array[write_pointer]=pointer;
                            write_pointer++;
                            memcpy(&fifo_array[write_pointer],&Successful_data_recieve_header,FIFO_ARRAY_MAX_STR_LEN); /// pass successful data receive header
                            write_pointer++;
                            pthread_cond_signal(&fifo_write);//wake up fpga read thread
                            pthread_mutex_unlock(&write_lock);
							//sem_post(&readerSem_fifoarray);
                            sec_heading=0;
                            heading=0;
                            count=0;
                            //pass pointer to write thread

                    }else{
                        processed_data_fifo_insert[count]=arr_copy[current_index];
                        count++;
                    }
                    goto while_end;
                }

        case 0b00000011:
            printf("inside master\n");
            break;
        case 0b00000100: /// XML data

            printf("inside xml\n");
            switch(sec_heading){
            case 0b00000000:
                sec_heading=arr_copy[current_index]<<4;
                local_len_count=0; /// count the received no of bytes in individual sections
                xml_name_received=0; /// boolean variable memorized the whether the boolean varible received
                goto while_end;
            case 0b00010000:/// save xml file
                if(xml_name_received){
                    if(arr_copy[current_index]==255){/// all data had received and release the heading and secaa_heading
                        sec_heading=0; /// reset the heading
                        heading =0; /// reset the heading
                        fclose(fptr); // c;ose the opened file
                        xml_name_received=0; // resets the variables
                        local_len_count=0; // resets the variables
                    }else{
                        /// data receiving. Should be written into the .txt file
                        /// fscanf(fptr,"%s\n",arr_copy[current_index]);
                        fputs(&arr_copy[current_index],fptr);
                    }
                }else{
                    xml_file_name[local_len_count]=arr_copy[current_index];
                    if(local_len_count==9){
                        xml_name_received=1;
                        // generate a new txt file which has the same name of the received data
                        fptr=fopen(xml_file_name,"w");
                    }else{
                        local_len_count++;
                    }
                }
                goto while_end;
            case 0b00100000:/// xml load
                /// Read the first line of the boot_config.txt file and send the xml file name if is it available.
                /// otherwise send the "xml not set" reply under the control commands
				fptr = fopen(boot_file_name, "r");
				if (fptr == NULL) {
					//XML error msg
					transmit_array[0] = 2;
					transmit_array[1] = 0b11101001;
					pthread_mutex_lock(&write_lock_tx);
					if((fifo_array_tx_writepointer==TX_FIFO_DECODER_SIZE && fifo_array_tx_readpointer==0)|| (fifo_array_tx_readpointer-fifo_array_tx_writepointer)==1){
                                /// FIFO is full
                        pthread_cond_wait(&fifo_read_tx,&write_lock_tx);//wait till data is read by reading thread
                    }

					memcpy(&fifo_array_tx[fifo_array_tx_writepointer], transmit_array, 2);
					fifo_array_tx_writepointer++;
					pthread_cond_signal(&fifo_write_tx);//wake up tx thread
                    pthread_mutex_unlock(&write_lock_tx);
					//sem_post(&readerSem_tx);

				}
				else {
					fseek(fptr, 10, SEEK_SET);
					if (fgetc(fptr)=='\n'|| fgetc(fptr) == '\n') {
						//xml not set error
						transmit_array[0] = 2;
						transmit_array[1] = 0b11100101;
						//sem_wait(&writerSem_tx);
						pthread_mutex_lock(&write_lock_tx);
                        if((fifo_array_tx_writepointer==TX_FIFO_DECODER_SIZE && fifo_array_tx_readpointer==0)|| (fifo_array_tx_readpointer-fifo_array_tx_writepointer)==1){
                                /// FIFO is full
                            pthread_cond_wait(&fifo_read_tx,&write_lock_tx);//wait till data is read by reading thread
                        }
						memcpy(&fifo_array_tx[fifo_array_tx_writepointer], transmit_array, 2);
						fifo_array_tx_writepointer++;
						pthread_cond_signal(&fifo_write_tx);//wake up tx thread
                         pthread_mutex_unlock(&write_lock_tx);
						//sem_post(&readerSem_tx);
					}
					else {
						transmit_array[0] =12;
						transmit_array[1] = 0b11100010;
						fseek(fptr, 11, SEEK_SET);
						fgets(&transmit_array[1], 11, fptr);
						//sem_wait(&writerSem_tx);
						pthread_mutex_lock(&write_lock_tx);
                        if((fifo_array_tx_writepointer==TX_FIFO_DECODER_SIZE && fifo_array_tx_readpointer==0)|| (fifo_array_tx_readpointer-fifo_array_tx_writepointer)==1){
                                /// FIFO is full
                            pthread_cond_wait(&fifo_read_tx,&write_lock_tx);//wait till data is read by reading thread
                        }
						memcpy(&fifo_array_tx[fifo_array_tx_writepointer], transmit_array,12);
						fifo_array_tx_writepointer++;
						pthread_cond_signal(&fifo_write_tx);//wake up tx thread
                        pthread_mutex_unlock(&write_lock_tx);
						//sem_post(&readerSem_tx);
					}
				}

                goto while_end;

            case 0b01000000:/// Read available
				l =2;
				DIR *p;

				p = opendir("./");

				if (p != NULL)
				{
					transmit_array[1] = 0b01000100;

					while ((pp = readdir(p)) != NULL) {
						int length = strlen(pp->d_name);
						if (strncmp(pp->d_name + length - 4, ".txt", 4) == 0) {

							if (strcmp(pp->d_name,boot_file_name ) != 0) {
								memcpy(xml_file_name, pp->d_name, 10);
								for (int i = 0; i < 10; i++) {
									transmit_array[l] = xml_file_name[i];
									l++;
									if (l == TX_FIFO_DECODER_MAX_STR_LEN) {
										transmit_array[0] = l;//upto which index we have to read data+1(including first byte)
										//sem_wait(&writerSem_tx);
										pthread_mutex_lock(&write_lock_tx);
                                        if((fifo_array_tx_writepointer==TX_FIFO_DECODER_SIZE && fifo_array_tx_readpointer==0)|| (fifo_array_tx_readpointer-fifo_array_tx_writepointer)==1){
                                            /// FIFO is full
                                            pthread_cond_wait(&fifo_read_tx,&write_lock_tx);//wait till data is read by reading thread
                                        }
										memcpy(&fifo_array_tx[fifo_array_tx_writepointer], transmit_array, TX_FIFO_DECODER_MAX_STR_LEN);
										fifo_array_tx_writepointer++;
										l = 1;
										pthread_cond_signal(&fifo_write_tx);//wake up tx thread
                                        pthread_mutex_unlock(&write_lock_tx);
										//sem_post(&readerSem_tx);
									}

								}

							}
						}
					}


					(void)closedir(p);
					if (l == TX_FIFO_DECODER_MAX_STR_LEN) {
						transmit_array[0] = l;
						//sem_wait(&writerSem_tx);
						pthread_mutex_lock(&write_lock_tx);
                        if((fifo_array_tx_writepointer==TX_FIFO_DECODER_SIZE && fifo_array_tx_readpointer==0)|| (fifo_array_tx_readpointer-fifo_array_tx_writepointer)==1){
                                            /// FIFO is full
                            pthread_cond_wait(&fifo_read_tx,&write_lock_tx);//wait till data is read by reading thread
                        }
						memcpy(&fifo_array_tx[fifo_array_tx_writepointer], transmit_array, TX_FIFO_DECODER_MAX_STR_LEN);
						fifo_array_tx_writepointer++;

						transmit_array[0] = 2;
						transmit_array[1] = 255;
						memcpy(&fifo_array_tx[fifo_array_tx_writepointer], transmit_array, 2);
						fifo_array_tx_writepointer++;
						//sem_post(&readerSem_tx);
						pthread_cond_signal(&fifo_write_tx);//wake up tx thread
                        pthread_mutex_unlock(&write_lock_tx);


					}else{
						transmit_array[0] =l;
						transmit_array[l] = 255;
						//sem_wait(&writerSem_tx);
						pthread_mutex_lock(&write_lock_tx);
                        if((fifo_array_tx_writepointer==TX_FIFO_DECODER_SIZE && fifo_array_tx_readpointer==0)|| (fifo_array_tx_readpointer-fifo_array_tx_writepointer)==1){
                                            /// FIFO is full
                            pthread_cond_wait(&fifo_read_tx,&write_lock_tx);//wait till data is read by reading thread
                        }
						memcpy(&fifo_array_tx[fifo_array_tx_writepointer], transmit_array,l);
						fifo_array_tx_writepointer++;
						//sem_post(&readerSem_tx);
						pthread_cond_signal(&fifo_write_tx);//wake up tx thread
                        pthread_mutex_unlock(&write_lock_tx);




					}
					l = 0;

				}


				else {
					transmit_array[0] = 2;
					transmit_array[1] = 0b11101001;
					//sem_wait(&writerSem_tx);
					pthread_mutex_lock(&write_lock_tx);
                    if((fifo_array_tx_writepointer==TX_FIFO_DECODER_SIZE && fifo_array_tx_readpointer==0)|| (fifo_array_tx_readpointer-fifo_array_tx_writepointer)==1){
                                            /// FIFO is full
                        pthread_cond_wait(&fifo_read_tx,&write_lock_tx);//wait till data is read by reading thread
                    }
					memcpy(&fifo_array_tx[fifo_array_tx_writepointer], transmit_array, 2);
					fifo_array_tx_writepointer++;
					//sem_post(&readerSem_tx);
					pthread_cond_signal(&fifo_write_tx);//wake up tx thread
                    pthread_mutex_unlock(&write_lock_tx);


				}
                /// Read the available xml file names within the folder. send the file names to the PC
                /// If there are no any xml file, then send "XML error" under control command
                goto while_end;

            case 0b00110000:/// XML Read
				l = 11;
                xml_file_name[local_len_count]=arr_copy[current_index];
                if(local_len_count==l){
                    heading=0; /// reset the heading
                    sec_heading=0; /// reset the heading
                    local_len_count=0; /// reset variables

                    strcpy(filename,xml_file_name);
                    strcat(filename,".txt");

                    /// Read the filename File and send the data to PC



					fptr = fopen(filename, "r");
					if (fptr == NULL) {
						//XML error msg
						transmit_array[0] =2;
						transmit_array[1] = 0b11101001;
						//sem_wait(&writerSem_tx);
						pthread_mutex_lock(&write_lock_tx);
                        if((fifo_array_tx_writepointer==TX_FIFO_DECODER_SIZE && fifo_array_tx_readpointer==0)|| (fifo_array_tx_readpointer-fifo_array_tx_writepointer)==1){
                                            /// FIFO is full
                            pthread_cond_wait(&fifo_read_tx,&write_lock_tx);//wait till data is read by reading thread
                        }
						memcpy(&fifo_array_tx[fifo_array_tx_writepointer], transmit_array,2);
						fifo_array_tx_writepointer++;
						//sem_post(&readerSem_tx);
						pthread_cond_signal(&fifo_write_tx);//wake up tx thread
                        pthread_mutex_unlock(&write_lock_tx);


					}
					transmit_array[1] = 0b01000001;
					memcpy(&transmit_array[2], xml_file_name, 10);

					while(1) {

						transmit_array[l] = fgetc(fptr);
						l++;
						if (feof(fptr)) {
							if (l == TX_FIFO_DECODER_MAX_STR_LEN) {
								transmit_array[0] = l;
								//sem_wait(&writerSem_tx);
								pthread_mutex_lock(&write_lock_tx);
                                if((fifo_array_tx_writepointer==TX_FIFO_DECODER_SIZE && fifo_array_tx_readpointer==0)|| (fifo_array_tx_readpointer-fifo_array_tx_writepointer)==1){
                                            /// FIFO is full
                                    pthread_cond_wait(&fifo_read_tx,&write_lock_tx);//wait till data is read by reading thread
                                }
								memcpy(&fifo_array_tx[fifo_array_tx_writepointer], transmit_array, TX_FIFO_DECODER_MAX_STR_LEN);
								fifo_array_tx_writepointer++;
								l = 1;
								transmit_array[l] = 255;
								memcpy(&fifo_array_tx[fifo_array_tx_writepointer], transmit_array,2);
								fifo_array_tx_writepointer++;
								l = 0;
                                pthread_cond_signal(&fifo_write_tx);//wake up tx thread
                                pthread_mutex_unlock(&write_lock_tx);
								//sem_post(&readerSem_tx);
								break;
							}
							else {

								transmit_array[l] = 255;
								//sem_wait(&writerSem_tx);
								pthread_mutex_lock(&write_lock_tx);
                                if((fifo_array_tx_writepointer==TX_FIFO_DECODER_SIZE && fifo_array_tx_readpointer==0)|| (fifo_array_tx_readpointer-fifo_array_tx_writepointer)==1){
                                            /// FIFO is full
                                    pthread_cond_wait(&fifo_read_tx,&write_lock_tx);//wait till data is read by reading thread
                                }
								memcpy(&fifo_array_tx[fifo_array_tx_writepointer], transmit_array,l+1);
								fifo_array_tx_writepointer++;
								l = 0;
								pthread_cond_signal(&fifo_write_tx);//wake up tx thread
                                pthread_mutex_unlock(&write_lock_tx);
								break;
								//sem_post(&readerSem_tx);
							}
						}
						if (l== TX_FIFO_DECODER_MAX_STR_LEN) {
							transmit_array[0] = l;
							//sem_wait(&writerSem_tx);
							pthread_mutex_lock(&write_lock_tx);
                            if((fifo_array_tx_writepointer==TX_FIFO_DECODER_SIZE && fifo_array_tx_readpointer==0)|| (fifo_array_tx_readpointer-fifo_array_tx_writepointer)==1){
                                            /// FIFO is full
                                pthread_cond_wait(&fifo_read_tx,&write_lock_tx);//wait till data is read by reading thread
                            }
							memcpy(&fifo_array_tx[fifo_array_tx_writepointer], transmit_array,TX_FIFO_DECODER_MAX_STR_LEN);
							fifo_array_tx_writepointer++;
							l = 0;
							//sem_post(&readerSem_tx);
							pthread_cond_signal(&fifo_write_tx);//wake up tx thread
                            pthread_mutex_unlock(&write_lock_tx);
						}


					}

					fclose(fptr);
                    /// If there are no such a xml file, then send "XML error" under control command
                }else{
                    local_len_count++;
                }

                goto while_end;

            case 0b01010000:/// set


                xml_file_name[local_len_count]=arr_copy[current_index]; /// load the XML name that needed to be set as  active XML file
                if(local_len_count==9){
                    local_len_count=0; // reset variables
                    heading=0; /// reset the heading
                    sec_heading=0; /// reset the heading
                    status_of_the_current_process=1; /// success

                    /// Check the availability to the xml_name file in the folder
					strcpy(filename, xml_file_name);
					strcat(filename, ".txt");
					int fd = access(filename, F_OK);
                    if (fd==-1){
                        status_of_the_current_process=0; /// Unsuccess
                    }


                    fptr=fopen("boot_config.txt","r+"); /// write the set XML file name into the boot_config.txt
                    if(fptr!=NULL){ /// FILE is AVAILABLE   !
                        for(int i=0;i<no_of_lines_config_txt;i++){
                            if(fgets(line[i], 40, fptr)!=NULL){
                                printf("%s",line[i]);
                            }

                        }
                        line[0][11]=xml_file_name[0];
                        line[0][12]=xml_file_name[1];
                        line[0][13]= xml_file_name[2];
                        line[0][14]= xml_file_name[3];
                        line[0][15]= xml_file_name[4];
                        line[0][16]= xml_file_name[5];
                        line[0][17]= xml_file_name[6];
                        line[0][18]= xml_file_name[7];
                        line[0][19]= xml_file_name[8];
                        line[0][20]= xml_file_name[9];
                        /// Write data into XML File
                        fseek(fptr,0,SEEK_SET);
                        for(int i=0;i<no_of_lines_config_txt;i++){
                            if(line[i]!=NULL){
                                fputs(line[i],fptr);
                            }
                        }
                        fclose(fptr);

                    }else{/// FILE is not AVAILABLE
                        fptr=fopen("boot_config.txt","w+");
                        line[0][11]= xml_file_name[0];
                        line[0][12]= xml_file_name[1];
                        line[0][13]= xml_file_name[2];
                        line[0][14]= xml_file_name[3];
                        line[0][15]= xml_file_name[4];
                        line[0][16]= xml_file_name[5];
                        line[0][17]= xml_file_name[6];
                        line[0][18]= xml_file_name[7];
                        line[0][19]= xml_file_name[8];
                        line[0][20]= xml_file_name[9];
                        //fseek(fptr,0,SEEK_SET);
                        for(int i=0;i<no_of_lines_config_txt;i++){
                            if(line[i]!=NULL){
                                fputs(line[i],fptr);
                            }
                        }
                        fclose(fptr);
                    }
                    if(status_of_the_current_process){
                        /// success
						transmit_array[0] = 12;
						transmit_array[1] = set_success_header;
						memcpy(&transmit_array[2], filename, 10);
						//sem_wait(&writerSem_tx);
						pthread_mutex_lock(&write_lock_tx);
                        if((fifo_array_tx_writepointer==TX_FIFO_DECODER_SIZE && fifo_array_tx_readpointer==0)|| (fifo_array_tx_readpointer-fifo_array_tx_writepointer)==1){
                                            /// FIFO is full
                            pthread_cond_wait(&fifo_read_tx,&write_lock_tx);//wait till data is read by reading thread
                        }
						memcpy(&fifo_array_tx[fifo_array_tx_writepointer],transmit_array, 12);
						fifo_array_tx_writepointer++;
						//sem_post(&readerSem_tx);
						pthread_cond_signal(&fifo_write_tx);//wake up tx thread
                        pthread_mutex_unlock(&write_lock_tx);

                    }else{
                        /// Unsuccess
						transmit_array[0] = 12;
						transmit_array[1] = set_unsuccess_header;
						memcpy(&transmit_array[2], filename, 10);
						//sem_wait(&writerSem_tx);
						pthread_mutex_lock(&write_lock_tx);
                        if((fifo_array_tx_writepointer==TX_FIFO_DECODER_SIZE && fifo_array_tx_readpointer==0)|| (fifo_array_tx_readpointer-fifo_array_tx_writepointer)==1){
                                            /// FIFO is full
                            pthread_cond_wait(&fifo_read_tx,&write_lock_tx);//wait till data is read by reading thread
                        }
						memcpy(&fifo_array_tx[fifo_array_tx_writepointer], transmit_array, 12);
						fifo_array_tx_writepointer++;
						//sem_post(&readerSem_tx);
						pthread_cond_signal(&fifo_write_tx);//wake up tx thread
                        pthread_mutex_unlock(&write_lock_tx);

                    }

                    /// Send the reply of the status of XML set process
                }else{
                     local_len_count++;
                }
                goto while_end;

            case 0b01100000:/// Delete
                xml_file_name[local_len_count]=arr_copy[current_index];


                if(local_len_count==9){
                    heading=0; /// reset the heading
                    sec_heading=0; /// reset the heading
                    local_len_count=0; /// reset variables
                    //status_of_the_current_process=1; /// success

                    strcpy(filename,xml_file_name);
                    strcat(filename,".txt");
                    if (!remove(filename)){
                        //printf("The file is Deleted successfully");
						transmit_array[0] = 12;
						transmit_array[1]=delete_success_header;
						memcpy(&transmit_array[2], filename,10);
						//sem_wait(&writerSem_tx);
						pthread_mutex_lock(&write_lock_tx);
                        if((fifo_array_tx_writepointer==TX_FIFO_DECODER_SIZE && fifo_array_tx_readpointer==0)|| (fifo_array_tx_readpointer-fifo_array_tx_writepointer)==1){
                                            /// FIFO is full
                            pthread_cond_wait(&fifo_read_tx,&write_lock_tx);//wait till data is read by reading thread
                        }
						memcpy(&fifo_array_tx[fifo_array_tx_writepointer], transmit_array, 12);
						fifo_array_tx_writepointer++;
						pthread_cond_signal(&fifo_write_tx);//wake up tx thread
                        pthread_mutex_unlock(&write_lock_tx);

						//sem_post(&readerSem_tx);
                    }else{
                       // printf("the file is not Deleted");
                        //status_of_the_current_process=0; /// unsuccess

						transmit_array[0] =12;
						transmit_array[1]=delete_unsuccess_header;
						memcpy(&transmit_array[2], filename,10);
						//sem_wait(&writerSem_tx);
						pthread_mutex_lock(&write_lock_tx);
                        if((fifo_array_tx_writepointer==TX_FIFO_DECODER_SIZE && fifo_array_tx_readpointer==0)|| (fifo_array_tx_readpointer-fifo_array_tx_writepointer)==1){
                                            /// FIFO is full
                            pthread_cond_wait(&fifo_read_tx,&write_lock_tx);//wait till data is read by reading thread
                        }
						memcpy(&fifo_array_tx[fifo_array_tx_writepointer], transmit_array, 12);
						fifo_array_tx_writepointer++;
						//sem_post(&readerSem_tx);
						pthread_cond_signal(&fifo_write_tx);//wake up tx thread
                        pthread_mutex_unlock(&write_lock_tx);

                    }
                    /*if(status_of_the_current_process){
                        /// success


                    }else{
                        /// Unsuccess
                    }*/

                }else{
                    local_len_count++;
                }

                goto while_end;
            }

//        case 0b00000101:
//            printf("inside master_control\n");
//
//            goto while_end;

        case 0b00000110:
            printf("inside control\n");
            break;
        case 0b00000111:
            printf("inside emergency\n");
            break;
        default:
            printf("inside reserved\n");
            break;


    }

    }
    while_end:
    current_index=current_index+1;
}

void FPGA_data_write(){
    pthread_mutex_lock(&read_lock);
	//sem_post(&readerSem_fifoarray);
    if(read_pointer==write_pointer){
        /// FIFO is empty
        pthread_cond_wait(&fifo_write,&read_lock);//wait till data is read by reading thread
    }
    ///fifo_array[read_pointer]= //required data;
    read_pointer++;
    pthread_cond_signal(&fifo_read);//wake up fpga read thread
    pthread_mutex_unlock(&read_lock);
	//sem_wait(&writerSem_fifoarray);


}


int main() {
    /// value assignment to the decoder_arg
	sem_init(&writerSem,0,DECODER_FIFO_SIZE-1);
	sem_init(&readerSem,0,0);

	//sem_init(&writerSem_fifoarray, 0, FIFO_ARRAY_SIZE - 1);
	//sem_init(&readerSem_fifoarray, 0, 0);

	//sem_init(&writerSem_tx, 0, TX_FIFO_DECODER_SIZE - 1);
	//sem_init(&readerSem_tx, 0, 0);

    for (int i=0; i<DECODER_FIFO_SIZE;i++){
        decoder_arg.pointer[i] = &fifo_array_rx[i][0];
        decoder_arg.length[i] = &length_rx[i];
        //decoder_arg.status_array[i] = &status_array_rx[i];
    }
    /*for (int i=0; i<TX_FIFO_DECODER_SIZE;i++){
        tx_arg.pointer_decoder[i] = &fifo_array_tx[i][0];

        //decoder_arg.status_array[i] = &status_array_rx[i];
    }

    for (int i=0; i<FPGA_READ_FIFO_SIZE;i++){
        tx_arg.pointer_fpga_write[i] = &fifo_array_FPGA_READ[i][0];

        //decoder_arg.status_array[i] = &status_array_rx[i];
    }*/

  // Open the serial port. Change device path as needed (currently set to an standard FTDI USB-UART cable type device)
  int serial_port = open("/dev/ttyS0", O_RDWR);


  // Create new termios struc, we call it 'tty' for convention
  struct termios tty;

  // Read in existing settings, and handle any error
  if(tcgetattr(serial_port, &tty) != 0) {
      printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
      return 1;
  }

  tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
  tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
  tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size
  tty.c_cflag |= CS8; // 8 bits per byte (most common)
  tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
  tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO; // Disable echo
  tty.c_lflag &= ~ECHOE; // Disable erasure
  tty.c_lflag &= ~ECHONL; // Disable new-line echo
  tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
  tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

  tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
  tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
  // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
  // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

  tty.c_cc[VTIME] = 0;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
  tty.c_cc[VMIN] = 5;

  // Set in/out baud rate to be 9600
  cfsetispeed(&tty, B115200);
  cfsetospeed(&tty, B115200);

  // Save tty settings, also checking for error
  if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
      printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
      return 1;
  }
pthread_t readThread_t, decoder_t,writeThread_t;  /* thread variables */
//
pthread_create(&readThread_t, NULL, (void *)readThread, (void *)&serial_port);
pthread_create(&writeThread_t, NULL, (void *)writeThread, (void *)&serial_port);
pthread_create(&decoder_t, NULL, (void *)decoder, (void *)&decoder_arg);
//
pthread_join(readThread_t, NULL);
pthread_join(writeThread_t, NULL);
pthread_join(decoder_t,NULL);

sem_destroy(&readerSem);
sem_destroy(&writerSem);
//sem_destroy(&readerSem_fifoarray);
//sem_destroy(&writerSem_fifoarray);

  close(serial_port);
  return 0; // success
}

