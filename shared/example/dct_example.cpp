#include "shared/misc.h"
#include "shared/dctquant16.h"
#include "shared/medianfilter.h"

int main(int argc,const char* argv[]) {
	unsigned int width=240;
	unsigned int height=180;

	FILE* fp=fopen(argv[1],"rb");
	if(!fp) {
		uprintf("Failed to open %s\n",argv[1]);
		return 1;
	}

	std::vector<uint16_t> in(height*width);
	size_t nread=fread(in.data(),sizeof(in[0]),in.size(),fp);
	fclose(fp);

	size_t in_size=sizeof(in[0])*in.size();
	uprintf("read %zu bytes\n",in_size);
	if(nread!=height*width) {
		uprintf("Wrong size, expected %dx%d (%zu Bpp)\n",width,height,sizeof(in[0]));
		return 1;
	}

	size_t out_size;
	EncoderDCT16 encoder;
	std::vector<uint8_t> out;

	out.clear();
	encoder.Encode(&out,in,width,height);
	out_size=sizeof(out[0])*out.size();
	uprintf("unfiltered data\n");
	uprintf("  encoded size: %zu\n",out_size);
	uprintf("  compression : %6.3f\%\n",100-100*(out_size*1.0/in_size));
	uprintf("  comp. ratio : %6.3f\n",(out_size*1.0/in_size));

	std::vector<uint16_t> filtered(height*width);
	MedianFilter3x3(filtered.data(),in.data(),width,height);

	out.clear();
	encoder.Encode(&out,filtered,width,height);
	out_size=sizeof(out[0])*out.size();
	uprintf("median filtered data\n");
	uprintf("  encoded size: %zu\n",out_size);
	uprintf("  compression : %6.3f\%\n",100-100*(out_size*1.0/in_size));
	uprintf("  comp. ratio : %6.3f\n",(out_size*1.0/in_size));
	return 0;
}
