#ifndef __KEY_H__
#define __KEY_H__

#include <stdbool.h>
#include <stdint.h>

struct key_desc;
typedef struct key_desc *key_desc_t;

void key_init(key_desc_t key);
bool key_read(key_desc_t key);

#endif /* __KEY_H__ */
