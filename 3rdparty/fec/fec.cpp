// Copyright (C) 2022 Storj Labs, Inc.
// See LICENSE for copying information.
// https://github.com/storj/infectious-cpp/blob/main/LICENSE

#include <algorithm>
#include <cstring>
#include <stdexcept>
#include <vector>
#include "fec.h"
#include "tables.h"

//namespace infectious {

// we go without bounds checking on accesses to the gf_mul_table
// and its friends.
// NOLINTBEGIN(cppcoreguidelines-pro-bounds-constant-array-index)

void FEC::initialize() {
	if (k <= 0 || n <= 0 || k > byte_max || n > byte_max || k > n) {
		throw std::domain_error("requires 1 <= k <= n <= 256");
	}

	std::vector<uint8_t> temp_matrix(n*k, 0);
	createInvertedVdm(temp_matrix, k);

	for (int i = k * k; i < n*k; i++) {
		temp_matrix[i] = gf_exp[((i/k)*(i%k))%(byte_max-1)];
	}

	for (int i = 0; i < k; i++) {
		enc_matrix[i*(k+1)] = 1;
	}

	for (int row = k * k; row < n*k; row += k) {
		for (int col = 0; col < k; ++col) {
			uint8_t acc {0};
			for (int i = 0; i < k; ++i) {
				const uint8_t row_v = temp_matrix[row + i];
				const uint8_t row_c = temp_matrix[col + k * i];
				acc ^= gf_mul_table[row_v][row_c];
			}
			enc_matrix[row+col] = acc;
		}
	}

	// vand_matrix has more columns than rows
	// k rows, n columns.
	vand_matrix[0] = 1;
	uint8_t g {1};
	for (int row = 0; row < k; row++) {
		uint8_t a {1};
		for (int col = 1; col < n; col++) {
			vand_matrix[row*n+col] = a; // 2.pow(i * j) FIGURE IT OUT
			a = gf_mul_table[g][a];
		}
		g = gf_mul_table[2][g];
	}
}

struct pivotSearcher {
	pivotSearcher(int k_)
		: k {k_}
		, ipiv(k)
	{}

	auto search(int col, const std::vector<uint8_t>& matrix) -> std::pair<int, int> {
		if (!ipiv[col] && matrix[col*k+col] != 0) {
			ipiv[col] = true;
			return std::make_pair(col, col);
		}

		for (int row = 0; row < k; row++) {
			if (ipiv[row]) {
				continue;
			}

			for (int i = 0; i < k; i++) {
				if (!ipiv[i] && matrix[row*k+i] != 0) {
					ipiv[i] = true;
					return std::make_pair(row, i);
				}
			}
		}

		throw std::invalid_argument("pivot not found");
	}

private:
	int k;
	std::vector<bool> ipiv;
};

// TODO(jeff): matrix is a K*K array, row major.
//
// NOLINTBEGIN(readability-function-cognitive-complexity)
void FEC::invertMatrix(std::vector<uint8_t>& matrix, int k) {
	pivotSearcher pivot_searcher(k);
	std::vector<int> indxc(k);
	std::vector<int> indxr(k);
	std::vector<uint8_t> id_row(k);

	for (int col = 0; col < k; col++) {
		auto [icol, irow] = pivot_searcher.search(col, matrix);

		if (irow != icol) {
			for (int i = 0; i < k; i++) {
				std::swap(matrix[irow*k+i], matrix[icol*k+i]);
			}
		}

		indxr[col] = irow;
		indxc[col] = icol;
		auto* pivot_row = &matrix[icol*k];
		auto c = pivot_row[icol];

		if (c == 0) {
			throw std::domain_error("singular matrix");
		}

		if (c != 1) {
			c = gf_inverse[c];
			pivot_row[icol] = 1;
			const auto& mul_c = gf_mul_table[c];

			for (int i = 0; i < k; i++) {
				pivot_row[i] = mul_c[pivot_row[i]];
			}
		}

		id_row[icol] = 1;
		if (std::memcmp(pivot_row, id_row.data(), k) != 0) {
			auto* p = matrix.data();
			for (int i = 0; i < k; i++) {
				if (i != icol) {
					c = p[icol];
					p[icol] = 0;
					addmul(&p[0], &p[k], pivot_row, c);
				}
				p = p + k;
			}
		}

		id_row[icol] = 0;
	}

	for (int i = 0; i < k; i++) {
		if (indxr[i] != indxc[i]) {
			for (int row = 0; row < k; row++) {
				std::swap(matrix[row*k+indxr[i]], matrix[row*k+indxc[i]]);
			}
		}
	}
}
// NOLINTEND(readability-function-cognitive-complexity)

void FEC::createInvertedVdm(std::vector<uint8_t>& vdm, int k) {
	if (k <= 1) {
		vdm[0] = 1;
		return;
	}

	std::vector<uint8_t> b(k);
	std::vector<uint8_t> c(k);

	c[k-1] = 0;
	for (int i = 1; i < k; i++) {
		const auto& mul_p_i = gf_mul_table[gf_exp[i]];
		for (int j = k - 1 - (i - 1); j < k-1; j++) {
			c[j] ^= mul_p_i[c[j+1]];
		}
		c[k-1] ^= gf_exp[i];
	}

	for (int row = 0; row < k; row++) {
		int index = 0;
		if (row != 0) {
			index = static_cast<int>(gf_exp[row]);
		}
		const auto& mul_p_row = gf_mul_table[index];

		uint8_t t = 1;
		b[k-1] = 1;
		for (int i = k - 2; i >= 0; i--) {
			b[i] = c[i+1] ^ mul_p_row[b[i+1]];
			t = b[i] ^ mul_p_row[t];
		}

		const auto& mul_t_inv = gf_mul_table[gf_inverse[t]];
		for (int col = 0; col < k; col++) {
			vdm[col*k+row] = mul_t_inv[b[col]];
		}
	}
}

// NOLINTEND(cppcoreguidelines-pro-bounds-constant-array-index)

//} // namespace infectious
