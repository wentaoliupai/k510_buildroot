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

#include "uv/unix.h"
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <memory.h>
#include <assert.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/signal.h>
#include <unistd.h>
#include <time.h>
#include <uv.h>
#define max(a,b) (a>b?a:b)

#define REGISTER_BASE_ADDR 0x92630000
#define MFBC_ADDR_START 0x92630000
#define MFBC_ADDR_END (0x006C + 0x0100 + MFBC_ADDR_START)
#define ISP_F2K_ADDR_START 0x92650000
#define ISP_F2K_ADDR_END (ISP_F2K_ADDR_START + 0x6000 + 0x017C)
#define ISP_R2K_ADDR_START 0x92660000
#define ISP_R2K_ADDR_END (ISP_R2K_ADDR_START + 0x6000 + 0x017C)
#define ISP_TOF_ADDR_START 0x92670000
#define ISP_TOF_ADDR_END (ISP_TOF_ADDR_START + 0x0400 + 0x0324)
#define REGISTER_END_ADDR (0x0324 + 0x0400 + 0x92670000)
#define DEFAULT_PORT 9982
#define PIC_YUV_SIZE 3133440

int page_size = 4096;
unsigned offset_in_page = 0;
unsigned map_size = 1048576;
void* map_base = MAP_FAILED;

uint32_t read_register(uint32_t addr) {
  if (addr < REGISTER_BASE_ADDR || addr > REGISTER_END_ADDR) {
    fprintf(stderr, "Read register 0x%08llX out of address\n", (unsigned long long)addr);
    return 0;
  }
#if QEMU
  const uint32_t val = addr;
#else
  const uint32_t val = *(volatile uint32_t*)((addr - REGISTER_BASE_ADDR) + map_base + offset_in_page);
#endif
  printf("Read register 0x%08llX, value: 0x%08llX\n", (unsigned long long)addr, (unsigned long long)val);
  return val;
}

void write_register(uint32_t addr, uint32_t value) {
  if (addr < REGISTER_BASE_ADDR || addr > REGISTER_END_ADDR) {
    fprintf(stderr, "Write register 0x%08llX out of address\n", (unsigned long long)addr);
    return;
  }
  printf("Write register 0x%08llX, value: 0x%08llX\n", (unsigned long long)addr, (unsigned long long)value);
#if QEMU
#else
  uint32_t* virt_addr = (addr - REGISTER_BASE_ADDR) + map_base + offset_in_page;
  *(volatile uint32_t*)virt_addr = value;
#endif
}

uv_loop_t *loop;
struct sockaddr_in addr;

struct list_node {
  struct list_node* next;
  struct list_node* prev;
};
struct list {
  struct list_node dummy_front;
  struct list_node dummy_back;
};

/**
 * @brief Init
 * @param l 
 */
void list_init(struct list* l) {
  l->dummy_front.next = &l->dummy_back;
  l->dummy_front.prev = NULL;
  l->dummy_back.next = NULL;
  l->dummy_back.prev = &l->dummy_front;
}

/**
 * @brief Insert new_node to the next position of pos
 * @param l 
 * @param node 
 */
void list_insert_back(struct list_node* pos, struct list_node* new_node) {
  new_node->prev = pos;
  new_node->next = pos->next;
  pos->next = new_node;
  new_node->next->prev = new_node;
}

/**
 * @brief Remove node from list
 * @param node 
 */
void list_remove(struct list_node* node) {
  node->prev->next = node->next;
  node->next->prev = node->prev;
}

#define list_push_front(l,n) list_insert_back(&l.dummy_front, n)
#define list_pop_back(l) list_remote(l.dummy_back.prev)
#define list_is_empty(l) (l.dummy_front.next == &l.dummy_back || l.dummy_back.prev == &l.dummy_front)

struct client_wrap {
  uv_tcp_t client;
  struct list_node node;
  uint8_t pic_done;
};

struct list_node* get_node_from_client(uv_stream_t* client) {
  struct client_wrap* w = (struct client_wrap*)client;
  return &w->node;
}
struct client_wrap* get_client_from_node(struct list_node* node) {
  return (struct client_wrap*)((uint8_t*)node - offsetof(struct client_wrap, node));
}

struct list clients;

void alloc_buffer(uv_handle_t *handle, size_t suggested_size, uv_buf_t *buf) {
  buf->base = (char*)malloc(suggested_size);
  buf->len = suggested_size;
}

// reuse buffer
struct broadcast_req {
  uint32_t ref_count;
  uint8_t buffer[];
};

struct wrap_uv_write {
  uv_write_t req;
  struct broadcast_req* b_req;
  uint8_t is_pic;
  struct client_wrap* client;
};

void sock_write(uv_write_t *req, int status) {
  if (status) {
    fprintf(stderr, "Write error %s\n", uv_strerror(status));
  }
  struct wrap_uv_write* w_req = (struct wrap_uv_write*)req;
  if (w_req->is_pic) {
    w_req->client->pic_done = 1;
  }
  w_req->b_req->ref_count--;
  if (w_req->b_req->ref_count == 0) {
    free(w_req->b_req);
  }
  free(w_req);
}

void broadcast(const void* buffer, size_t len, uint8_t is_pic) {
  if (list_is_empty(clients)) {
    return;
  }
  struct broadcast_req* b_req = malloc(4 + len);
  b_req->ref_count = 0;
  memcpy(b_req->buffer, buffer, len);
  struct list_node* node = clients.dummy_front.next;
  while (node != &clients.dummy_back) {
    struct client_wrap* client = get_client_from_node(node);
    // stream control
    if (client->pic_done || !is_pic) {
      if (is_pic) {
        client->pic_done = 0;
      }
      struct wrap_uv_write* req = malloc(sizeof(struct wrap_uv_write));
      req->client = client;
      req->is_pic = is_pic;
      req->b_req = b_req;
      uv_buf_t buf = uv_buf_init((char*)b_req->buffer, len);
      b_req->ref_count++;
      uv_write(&req->req, (uv_stream_t*)&client->client, &buf, 1, sock_write);
    } else {
      // drop frame
    }
    node = node->next;
  }
  if (b_req->ref_count == 0) {
    free(b_req);
  }
}

struct wrap_file_write {
  uv_fs_t open_req;
  uv_fs_t write_req;
  uv_fs_t close_req;
  uv_buf_t uv_buf;
  char buffer[];
};

void on_write(uv_fs_t* write_req) {
  // close file
  struct wrap_file_write* req = (struct wrap_file_write*)((void*)write_req - offsetof(struct wrap_file_write, write_req));
  if (write_req->result < 0) {
    fprintf(stderr, "Write error: %s\n", uv_strerror((int)write_req->result));
  }
  // close
  uv_fs_close(uv_default_loop(), &req->close_req, req->open_req.result, NULL);
  // clean
  uv_fs_req_cleanup(&req->close_req);
  uv_fs_req_cleanup(write_req);
  uv_fs_req_cleanup(&req->open_req);
  free(req);
}

void on_open(uv_fs_t* open_req) {
  struct wrap_file_write* req = (struct wrap_file_write*)open_req;
  if (open_req->result >= 0) {
    // write buffer
    uv_fs_write(uv_default_loop(), &req->write_req, open_req->result, &req->uv_buf, 1, 0, on_write);
  } else {
    fprintf(stderr, "error opening file %s\n", uv_strerror((int)open_req->result));
    // clean
    uv_fs_req_cleanup(open_req);
    free(req);
  }
}

#define MESSAGE_HEAD 0x99
#define CMD_READ 0x34
#define CMD_WRITE 0x12
#define CMD_PAUSE 0x9A
#define CMD_FILE 0xBC
uint8_t state = 0;
uint8_t* data;
uint32_t data_cap = 0;
uint32_t data_itr = 0;
uint32_t data_len = 0;
uint8_t flag_send_pic = 1;
void exec_cmd() {
  uint8_t cmd = *data;
  uint8_t* raw = data + 5;
  uint32_t raw_len = data_len - 5;
  uint32_t* blocks = (uint32_t*)raw;
  if (cmd == CMD_READ) {
    // broadcast to all client
    uint32_t block_num = raw_len / 4;
    const uint32_t write_len = 6 + block_num * 2 * 4;
    // FIXME: stack might overflow, expand stack size or use heap
    uint8_t write_buffer[write_len];
    memcpy(write_buffer, "\x99\x78", 2);
    *(uint32_t*)(write_buffer + 2) = block_num * 2 * 4;
    uint32_t* data_buffer = (uint32_t*)(write_buffer + 6);
    for (uint32_t i = 0; i < block_num; i++) {
      const uint32_t address = blocks[i];
      data_buffer[i * 2] = address;
      data_buffer[i * 2 + 1] = read_register(address);
    }
    broadcast(write_buffer, write_len, 0);
  } else if (cmd == CMD_WRITE) {
    uint32_t block_num = raw_len / 8;
    const uint32_t write_len = 6 + block_num * 2 * 4;
    uint8_t write_buffer[write_len];
    memcpy(write_buffer, "\x99\x78", 2);
    *(uint32_t*)(write_buffer + 2) = block_num * 2 * 4;
    uint32_t* data_buffer = (uint32_t*)(write_buffer + 6);
    for (uint32_t i = 0; i < block_num; i++) {
      const uint32_t address = blocks[i * 2];
      const uint32_t value = blocks[i * 2 + 1];
      write_register(address, value);
      data_buffer[i * 2] = address;
      data_buffer[i * 2 + 1] = read_register(address);
    }
    broadcast(write_buffer, write_len, 0);
  } else if (cmd == CMD_PAUSE) {
    if (raw_len != 1) {
      fprintf(stderr, "Wrong data length (%d) for pause command\n", raw_len);
      return;
    }
    if (*raw == 1) {
      flag_send_pic = 1;
      puts("Continue piture.");
    } else if (*raw == 0) {
      flag_send_pic = 0;
      puts("Pause piture.");
    } else {
      fprintf(stderr, "Wrong data (%d) for pause command\n", *raw);
    }
  } else if (cmd == CMD_FILE) {
    struct {
      uint32_t ser;
      uint32_t path_len;
      uint32_t file_len;
      uint8_t data[];
    }* file = raw;
    char path[file->path_len + 1];
    path[file->path_len] = '\0';
    memcpy(path, file->data, file->path_len);
    struct wrap_file_write* req = malloc(sizeof(struct wrap_file_write) + file->file_len);
    // copy buffer
    memcpy(req->buffer, file->data + file->path_len, file->file_len);
    req->uv_buf = uv_buf_init(req->buffer, file->file_len);
    printf("save file to %s, length: %d\n", path, file->file_len);
    uv_fs_open(uv_default_loop(), &req->open_req, path, O_CREAT | O_WRONLY | O_TRUNC, 0, on_open);
  } else {
    // unknown cmd
    fprintf(stderr, "Unknown command: 0x%02X\n", cmd);
  }
}

void message_update(uint8_t* buffer, size_t len) {
  for (size_t i = 0; i < len; i++) {
    uint8_t byte = buffer[i];
    switch (state) {
      case 0:
        if (byte == MESSAGE_HEAD) {
          data_itr = 0;
          state = 1;
        }
        break;
      case 1:
        // recv 5 bytes
        data[data_itr++] = byte;
        if (data_itr >= 5) {
          data_len = *(uint32_t*)(data + 1) + 5;
          if (data_len == 0) {
            // exec
            exec_cmd();
            state = 0;
            break;
          }
          // check capacity
          if (data_len > data_cap) {
            data = realloc(data, data_len);
            data_cap = data_len;
          }
          // TODO: shrink capacity, reduce memory leak
          state = 2;
        }
        break;
      case 2:
        data[data_itr++] = byte;
        if (data_itr >= data_len) {
          // exec cmd
          exec_cmd();
          state = 0;
        }
        break;
      default:
        state = 0;
        break;
    }
  }
}

void sock_close(uv_handle_t *handle) {
  if (handle != NULL) {
    free(handle);
  }
}

void sock_read(uv_stream_t *client, ssize_t nread, const uv_buf_t *buf) {
    if (nread < 0) {
      // remove from list
      struct list_node* n = get_node_from_client(client);
      list_remove(n);
      if (nread != UV_EOF) {
        fprintf(stderr, "Read error %s\n", uv_err_name(nread));
      } else {
        printf("Close connection, fd: %d\n", client->io_watcher.fd);
      }
      uv_close((uv_handle_t*) client, sock_close);
    } else if (nread > 0) {
      // TODO: isolate clients
      message_update((uint8_t *)buf->base, nread);
    }
    if (buf->base) {
      free(buf->base);
    }
}

void on_new_connection(uv_stream_t *server, int status) {
    if (status < 0) {
      fprintf(stderr, "New connection error %s\n", uv_strerror(status));
      return;
    }
    struct client_wrap* cw = malloc(sizeof(struct client_wrap));
    cw->pic_done = 1;
    uv_tcp_t *client = &cw->client;
    struct list_node* n = &cw->node;
    uv_tcp_init(loop, client);
    if (uv_accept(server, (uv_stream_t*) client) == 0) {
      uv_read_start((uv_stream_t*)client, alloc_buffer, sock_read);
      // insert to clients
      list_push_front(clients, n);
      printf("New connection, fd: %d\n", client->io_watcher.fd);
    } else {
      uv_close((uv_handle_t*) client, NULL);
    }
}

uint8_t pic_write_buffer[PIC_YUV_SIZE + 6] = {
  0x99, 0x9a,
  PIC_YUV_SIZE & 0xff,
  (PIC_YUV_SIZE >> 8) & 0xff,
  (PIC_YUV_SIZE >> 16) & 0xff,
  (PIC_YUV_SIZE >> 24) & 0xff
};
void stdin_read(uv_stream_t *stream, ssize_t nread, const uv_buf_t* buf)
{
  if (nread < 0) {
    uv_close((uv_handle_t*)stream, NULL);
    return;
  }
  if (flag_send_pic == 0) {
    // do not send
    return;
  }
  uint8_t* pic_nv12_buffer = pic_write_buffer + 6;
  static uint32_t ptr = 0;
  size_t lack = PIC_YUV_SIZE - ptr;
#if DEBUG_STDIN
  // send any way
  if (nread >= lack) {
    memcpy(pic_nv12_buffer + ptr, buf->base, lack);
  } else {
    memcpy(pic_nv12_buffer + ptr, buf->base, nread);
  }
  // broadcast
  broadcast(pic_write_buffer, sizeof(pic_write_buffer), 1);
  static uint32_t pic_num = 0;
  printf("\rpicture sent, #%d", pic_num++);
  fflush(stdout);
  ptr = 0;
#else
  if (nread >= lack) {
    static time_t last_time = 0;
    if (time(NULL) - last_time >= 1) {
      memcpy(pic_nv12_buffer + ptr, buf->base, lack);
      // broadcast
      broadcast(pic_write_buffer, sizeof(pic_write_buffer), 1);
      static uint32_t pic_num = 0;
      printf("\rpicture sent, #%d", pic_num++);
      fflush(stdout);
      last_time = time(NULL);
    }
    ptr = 0;
  } else {
    memcpy(pic_nv12_buffer + ptr, buf->base, nread);
    ptr += nread;
  }
#endif
  if (buf->base) {
    free(buf->base);
  }
}

void idle_mock_stdin (uv_idle_t* idle) {
  uv_buf_t buf = {
    .base = malloc(4),
    .len = 4
  };
  stdin_read(NULL, 4, &buf);
}

void before_exit(int sig) {
  uv_loop_close(loop);
  fprintf(stderr, "capture signal %d, exit\n", sig);
  exit(sig);
}

int main(int argc, char* argv[]) {
#if QEMU
  puts("In emulator, will not try to set register");
#else
  // map memory
  page_size = getpagesize();
  offset_in_page = (unsigned)REGISTER_BASE_ADDR & (page_size - 1);
  if (offset_in_page + 32 > page_size) {
    map_size += page_size;
	}
  printf("page size: %d\noff_t size: %ld\n"
         "offset_in_page: %d\nmap_size: %d\n",
         page_size, sizeof(off_t), offset_in_page, map_size);
  int mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
  assert(mem_fd >= 0);
  map_base = mmap(NULL, map_size, PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, REGISTER_BASE_ADDR & ~(off_t)(page_size - 1));
  assert(map_base != MAP_FAILED);
#endif

  // init
  data = malloc(14);
  data_cap = 14;
  list_init(&clients);

  // socket
  loop = uv_default_loop();
  signal(SIGINT, before_exit);
  signal(SIGTERM, before_exit);
  signal(SIGPIPE, SIG_IGN);
  uv_tcp_t server;
  uv_tcp_init(loop, &server);
  uv_ip4_addr("0.0.0.0", DEFAULT_PORT, &addr);
  uv_tcp_bind(&server, (const struct sockaddr*)&addr, 0);
  assert(setsockopt(server.io_watcher.fd, SOL_SOCKET, SO_REUSEADDR, &(int){1}, sizeof(int)) == 0);
  assert(uv_listen((uv_stream_t*)&server, 128, on_new_connection) == 0);

  uv_tty_t input;
  uv_handle_type t = uv_guess_handle(STDIN_FILENO);
  if (t == UV_TTY) {
    
  } else if (t == UV_FILE) {
    // TODO: read from file
    printf("stdin is not stdin\n");
  }
  uv_tty_init(loop, &input, STDIN_FILENO, 1);
  uv_read_start((uv_stream_t*)&input, alloc_buffer, stdin_read);
#if DEBUG_STDIN
  uv_idle_t idle_task;
  uv_idle_init(loop, &idle_task);
  uv_idle_cb c;
  uv_idle_start(&idle_task, idle_mock_stdin);
#endif
  return uv_run(loop, UV_RUN_DEFAULT);
}
