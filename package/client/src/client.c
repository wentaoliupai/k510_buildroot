/* Copyright (c) 2022, Canaan Bright Sight Co., Ltd
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include "client.h"

#define device_id "000000000835721845"
#define version_info "000000000000000108"
#define header_len 10
char* server_ip = NULL; 
typedef struct package_head{
	char length[4];
	char func_no[2];
    char seq_no[4];    

}package_head_t __attribute__((packed));

int assemble_packet(unsigned char *data, int func_no,int dlen,int num)
{
	package_head_t head;
	unsigned char 	*ptr = data;
	unsigned int	data_len = 0;

	memset(&head, 0x00, sizeof(head));
	sprintf(&head,"%04d%02d%04d",(dlen + header_len),func_no,num);
	memcpy(ptr, &head, sizeof(package_head_t));
	ptr += sizeof(package_head_t);
	data_len += sizeof(package_head_t);

	return data_len;
}

int parser_packet(uint8_t *data,int *len,int *num)
{
	uint8_t		*ptr = data;
	package_head_t  head;
    int fun_no = 0;
	uint8_t *header_func= (uint8_t*)malloc(sizeof(uint8_t)*2);
	uint8_t *header_seq= (uint8_t*)malloc(sizeof(uint8_t)*4);
	uint8_t *header_length= (uint8_t*)malloc(sizeof(uint8_t)*4);

	strncpy((char *)header_length,ptr,4);
	strncpy((char *)header_func,ptr + 4,2);
	strncpy((char *)header_seq,ptr + 6,4);

	*len = atoi(header_length);
    *num = atoi(header_seq);
    fun_no = atoi(header_func);
	
	free(header_func);
	free(header_seq);
	free(header_length);
	return fun_no;

}

char* despace(char *a){
	int i=0;char *b=a;
	while(a[i++]!='\0'){
		if(a[i]==' '){
			for(int j=i;j>=1;j--){
				a[j]=a[j-1];
			}
			b++;
		}
	}
	return b;
}

void update_basic_para(char *buff)
{
    char *room_number = (char*)malloc(sizeof(char)*4);
    char *logo_img_url = (char*)malloc(sizeof(char)*100);
    char *back_url = (char*)malloc(sizeof(char)*100);

    strncpy(room_number,buff + 28,4);
    strncpy(logo_img_url,buff + 32,100);
    strncpy(back_url,buff + 132,100);
	
    int number = atoi(room_number);
	printf("%d-%s-%s\n",number,despace(logo_img_url),despace(back_url));
    free(room_number);
	free(logo_img_url);
	free(back_url);
}

void set_lockbell_pic(char *buff)
{
    char *doorbell_img_url = (char*)malloc(sizeof(char)*100);
    char *lockbutton_img_url = (char*)malloc(sizeof(char)*100);

    strncpy(doorbell_img_url,buff + 28,100);
    strncpy(doorbell_img_url,buff + 128,100);
	
	printf("%%s-%s\n",despace(doorbell_img_url),despace(lockbutton_img_url));

	free(doorbell_img_url);
	free(lockbutton_img_url);
}

void set_roomstatus_pic(char *buff)
{
	char num = 0;
    char *roomstatus_1_img_url_a = (char*)malloc(sizeof(char)*100);
    char *roomstatus_1_img_url_b = (char*)malloc(sizeof(char)*100);
	char *roomstatus_2_img_url_a = (char*)malloc(sizeof(char)*100);
    char *roomstatus_2_img_url_b = (char*)malloc(sizeof(char)*100);
	char *roomstatus_3_img_url_a = (char*)malloc(sizeof(char)*100);
    char *roomstatus_3_img_url_b = (char*)malloc(sizeof(char)*100);
	char *roomstatus_4_img_url_a = (char*)malloc(sizeof(char)*100);
    char *roomstatus_4_img_url_b = (char*)malloc(sizeof(char)*100);

	strncpy((char*)&num,buff + 28,1);
    strncpy(roomstatus_1_img_url_a,buff + 29,100);
    strncpy(roomstatus_1_img_url_b,buff + 129,100);
	strncpy(roomstatus_2_img_url_a,buff + 229,100);
    strncpy(roomstatus_2_img_url_b,buff + 329,100);
	strncpy(roomstatus_3_img_url_a,buff + 429,100);
    strncpy(roomstatus_3_img_url_b,buff + 529,100);
	strncpy(roomstatus_4_img_url_a,buff + 629,100);
    strncpy(roomstatus_4_img_url_b,buff + 729,100);
	
	printf("%%s-%s\n",despace(roomstatus_2_img_url_b),despace(roomstatus_2_img_url_b));

	free(roomstatus_1_img_url_a);
	free(roomstatus_1_img_url_b);
	free(roomstatus_2_img_url_a);
	free(roomstatus_2_img_url_b);
	free(roomstatus_3_img_url_a);
	free(roomstatus_3_img_url_b);
	free(roomstatus_4_img_url_a);
	free(roomstatus_4_img_url_b);
}

char * update_roomstatus(char *buff)
{
	char *roomstatus = (char*)malloc(sizeof(char)*4);
	strncpy(roomstatus,buff + 28,4);
	printf("%%s\n",roomstatus);
}


void add_face_info(char *buff)
{
    char *personid = (char*)malloc(sizeof(char)*10);
    char *person_img_url = (char*)malloc(sizeof(char)*100);
	char *validity_time = (char*)malloc(sizeof(char)*14);
    char *cardno = (char*)malloc(sizeof(char)*8);
	char *validity_in_time = (char*)malloc(sizeof(char)*14);

    strncpy(personid,buff + 28,10);
    strncpy(person_img_url,buff + 38,100);
	strncpy(validity_time,buff + 138,14);
    strncpy(cardno,buff + 152,8);
	strncpy(validity_in_time,buff + 160,14);

	
	printf("%s-%s-%s\n",despace(person_img_url),personid,validity_in_time);

	free(personid);
	free(person_img_url);
	free(validity_time);
	free(cardno);
	free(validity_in_time);
}

void device_reset(char *buff)
{
	char *reset_time = (char*)malloc(sizeof(char)*14);
	strncpy(reset_time,buff + 28,14);
	free(reset_time);
}



int equipment_certify(uint8_t * data)
{

    char seq_no = 4;
    char func_no = 2;
    char len = 36;
    assemble_packet(data,func_no,len,seq_no);
    sprintf((char*)data + header_len,"%s%s",device_id,version_info);
}


int main(int argc, char **argv)
{
	//define var for thread
	if(argc < 2)
	{
		printf("please input server ip\n");
	}
	server_ip = argv[1];
	pthread_t thread_receive_id[CLIENT_MAX_NUM];
	
	printf("Creating thread..\n");
	for(int id_num = 0; id_num < 1; ++id_num) {
		if(pthread_create(&thread_receive_id[id_num], NULL, thread_receive, (void *)&id_num) != 0) {
			perror("Thread create failed");
			exit(EXIT_FAILURE);
		}
	}

	printf("Thread create finished. Test running...\n");
	while(1) {
		sleep(10);
		printf("Main process.\n");
	}

	printf("Client process exit.\n");
	exit(EXIT_SUCCESS);
}


void *thread_receive(void *arg)
{
	int seq_num = *(int *)arg;
	int local_sockfd = 0, local_len = 0;
	struct sockaddr_in local_address;
	uint8_t send_buff[100];
	int rd_res = 0;
	int length,num;
	char *recv_buff = NULL;

	if((local_sockfd = socket(AF_INET, SOCK_STREAM, 0 )) < 0) {
		perror("Create socket failed:");
		exit(EXIT_FAILURE);
	}

	local_address.sin_family = AF_INET;
	local_address.sin_addr.s_addr = inet_addr(server_ip);
	local_address.sin_port = htons(TEST_PORT);
	local_len = sizeof(local_address);

	if((recv_buff = malloc_packet()) == NULL) {
		printf("Malloc receive buff error.\n");
		pthread_exit("");
	}
	memset(recv_buff, 0, MTU_NUM);

	if(connect(local_sockfd, (struct sockaddr *)&local_address, local_len) < 0) {
		perror("Connect error");
		free(recv_buff);
		pthread_exit("");
	}
	equipment_certify(send_buff);
	//printf("Connect successfully,seq_num:%d, id:%ld\n", seq_num, pthread_self());
	write(local_sockfd,send_buff,46);
	while(1) {
		rd_res = recv(local_sockfd, recv_buff, MTU_NUM, 0);
		if(rd_res == 0) {
			printf("Disconnected, thread seq:%d", seq_num);
			break;
		}
		else if(rd_res < 0) {
			perror("Receive error");
			break;
		}
		printf("recv:%s\n",recv_buff);
		int func_no = parser_packet(recv_buff,&length,&num);
		printf("func_no %d\n",func_no);
		if(func_no == 11)
		{
			update_basic_para(recv_buff);
		}
		else if(func_no == 16)
		{
			add_face_info(recv_buff);
		}
		printf("%d %d %d \n",func_no,length,num);
		memset(recv_buff,0,MTU_NUM);
	}
	
	
	free(recv_buff);
	printf("Thread exit, seq:%d\n", seq_num);
	pthread_exit("");
}

char *malloc_packet()
{
	char *malloc_res = NULL;

	malloc_res = (char *)malloc(MTU_NUM * sizeof(char));
	if(malloc_res == NULL) {
		return (char *)NULL;
	}

	memset(malloc_res, 0, MTU_NUM);
	return malloc_res;
}