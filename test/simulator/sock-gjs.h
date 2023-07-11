#ifndef SOCK_GJS_H
#define SOCK_GJS_H

#include <stddef.h>

size_t handle_gjs_request(int fd, uint8_t *buf, size_t len, size_t buff_len_max);

#endif
