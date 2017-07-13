#ifndef UR_DRIVER_COMMON_H
#define UR_DRIVER_COMMON_H

#include <assert.h>
#include <netinet/in.h>
#include <stdlib.h>
#include <string.h>
#include <array>
#include <cstdint>
#include <memory>
#include <vector>

typedef std::array<double, 6> Vector6;
const Vector6 ZERO_VELOCITY = { 0., 0., 0., 0., 0., 0. };

inline void ToStdVector(const Vector6& in, std::vector<double>& out) {
	out.resize(6);
	for (int i = 0; i < 6; i++)
		out[i] = in[i];
}

inline void ToVector6(const std::vector<double>& in, Vector6& out) {
	assert(in.size() == 6);
	for (int i = 0; i < 6; i++)
		out[i] = in[i];
}

inline std::vector<double> ToStdVector(const Vector6& in) {
	std::vector<double> out(6);
	for (int i = 0; i < 6; i++)
		out[i] = in[i];
	return out;
}

inline Vector6 ToVector6(const std::vector<double>& in) {
	assert(in.size() == 6);
	Vector6 out;
	for (int i = 0; i < 6; i++)
		out[i] = in[i];
	return out;
}

inline void unpackVariable(uint8_t* buf, unsigned int& offset, double& out) {
	uint64_t unpack_to;
	memcpy(&unpack_to, &buf[offset], sizeof(unpack_to));
	offset += sizeof(unpack_to);
	unpack_to = be64toh(unpack_to);
	memcpy(&out, &unpack_to, sizeof(unpack_to));
}

inline void unpackVariable(uint8_t* buf, unsigned int& offset, float& out) {
	uint32_t unpack_to;
	memcpy(&unpack_to, &buf[offset], sizeof(unpack_to));
	offset += sizeof(unpack_to);
	unpack_to = be32toh(unpack_to);
	memcpy(&out, &unpack_to, sizeof(unpack_to));
}

inline void unpackVariable(uint8_t* buf, unsigned int& offset, int8_t& out) {
	memcpy(&out, &buf[offset], sizeof(int8_t));
	offset += sizeof(int8_t);
}

inline bool unpackBool(uint8_t* buf, unsigned int& offset) {
	uint8_t tmp;
	memcpy(&tmp, &buf[offset], sizeof(uint8_t));
	offset += sizeof(uint8_t);
	return tmp > 0;
}

inline void unpackVariable(uint8_t* buf, unsigned int& offset, uint8_t& out) {
	memcpy(&out, &buf[offset], sizeof(uint8_t));
	offset += sizeof(uint8_t);
}

inline void unpackVariable(uint8_t* buf, unsigned int& offset, int16_t& out) {
	memcpy(&out, &buf[offset], sizeof(int16_t));
	out = ntohs(out);
	offset += sizeof(int16_t);
}

inline void unpackVariable(uint8_t* buf, unsigned int& offset, uint16_t& out) {
	memcpy(&out, &buf[offset], sizeof(uint16_t));
	out = ntohs(out);
	offset += sizeof(uint16_t);
}

inline void unpackVariable(uint8_t* buf, unsigned int& offset, int32_t& out) {
	memcpy(&out, &buf[offset], sizeof(int32_t));
	out = ntohl(out);
	offset += sizeof(int32_t);
}

inline void unpackVariable(uint8_t* buf, unsigned int& offset, uint32_t& out) {
	memcpy(&out, &buf[offset], sizeof(uint32_t));
	out = ntohl(out);
	offset += sizeof(uint32_t);
}

template <size_t SIZE>
inline void unpackVector(uint8_t* buf, unsigned int& offset,
                         std::array<double, SIZE>& output) {
	double out;
	for (int i = 0; i < SIZE; i++) {
		unpackVariable(buf, offset, out);
		output[i] = out;
	}
}

#endif  // UR_DRIVER_COMMON_H
