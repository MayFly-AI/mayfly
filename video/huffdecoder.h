#pragma once

class Dict;
class FrameDecoder;
FrameDecoder* CreateHuffDecoder(const Dict* cfg);
