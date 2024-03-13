
#include <vector>
#include <stdint.h>
#include "3rdparty/stb/stb_image_write.h"

#include "shared/stb_ext.h"

typedef struct {
	std::vector<char>* data;
} custom_stbi_mem_context;

// custom write function
static void custom_stbi_write_mem(void *context, void *data, int size) {
	custom_stbi_mem_context *c = (custom_stbi_mem_context*)context; 
	c->data->insert(c->data->end(),(uint8_t*)data,(uint8_t*)data+size);
}

bool WriteJPGToMemory(std::vector<char>* data,uint8_t* pixels,int m_width,int m_height,int channels) {
	custom_stbi_mem_context context;
	context.data=data;
	int result = stbi_write_jpg_to_func(custom_stbi_write_mem, &context, m_width, m_height, channels, pixels, 50);
	return result ? true:false;
}
