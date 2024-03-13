#pragma once

#include <functional>
#include <string>

//namespace infectious {
	class FEC {
		public:
			FEC(int k_, int n_)	: k {k_}, n {n_}, enc_matrix(n*k, 0), vand_matrix(k*n, 0) {
				initialize();
			}
			static const int byte_max = 256;
			static void invertMatrix(std::vector<uint8_t>& matrix, int k);
			static void createInvertedVdm(std::vector<uint8_t>& vdm, int k);
			static void addmul(uint8_t* z_begin, const uint8_t* z_end,const uint8_t* x, uint8_t y);
			void correct_(std::vector<uint8_t*>& shares_vec, std::vector<int>& shares_nums, long share_size)const;
			void initialize();
			int k;
			int n;
			std::vector<uint8_t> enc_matrix;
			std::vector<uint8_t> vand_matrix;
	};
//};
