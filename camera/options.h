#pragma once

struct Options {
	float framerate{30.0f};

	//unsigned int width{1640};
	//unsigned int height{1232};

	unsigned int width{1280};
	unsigned int height{720};

	//unsigned int width{640};
	//unsigned int height{480};

	uint64_t timeout{0}; // in ms, 0 means never
	//uint64_t exposure{20000}; // in us, 0 means auto
	uint64_t exposure{0}; // in us, 0 means auto

	unsigned int camera{0};
	float ev{0};

	float brightness{0.0f};
	float contrast{1.0f};
	float saturation{1.0f};
	float sharpness{1.0f};

	bool inline_headers{true};
	bool synchronize{true};
	int64_t synchronize_offset{0};

	std::string codec{"h264"};
	std::string profile{"high"};
	std::string level{"4.2"};
	//unsigned int bitrate{0};
	unsigned int bitrate{4000000};
};
