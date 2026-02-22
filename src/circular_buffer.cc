/**
 * @file circular_buffer.cc
 * @brief Implementation of the Magic Ring Buffer (Windows/Linux/MacOS).
 */

/*
 * @author Joshua Lackey (original)
 * @author Benjamin Vernoux <bvernoux@hydrasdr.com> (improvements)
 * @copyright 2010 Joshua Lackey, 2025 Benjamin Vernoux
 * @license BSD-2-Clause
 */

#ifdef _WIN32
#include "win_compat.h"
#endif

#include "circular_buffer.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <climits>
#include <stdexcept>
#include <algorithm>

#ifndef _MSC_VER
#include <unistd.h>
#endif
#ifndef _WIN32
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#endif

#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif

// ----------------------------------------------------------------------------
// Windows Implementation
// ----------------------------------------------------------------------------
#ifdef _WIN32
circular_buffer::circular_buffer(unsigned int buf_len, unsigned int item_size, int overwrite) {
	if (!buf_len) throw std::runtime_error("circular_buffer: buffer len is 0");
	if (!item_size) throw std::runtime_error("circular_buffer: item size is 0");

	m_item_size = item_size;
	m_overwrite = overwrite;

	/* Check for multiplication overflow before allocating */
	if (buf_len > UINT_MAX / item_size)
		throw std::runtime_error("circular_buffer: buffer size overflow");

	SYSTEM_INFO sysInfo;
	GetSystemInfo(&sysInfo);
	unsigned int granularity = sysInfo.dwAllocationGranularity;

	unsigned int raw_size = item_size * buf_len;
	m_buf_size = (raw_size + granularity - 1) & ~(granularity - 1);
	m_buf_len = m_buf_size / item_size;

	d_handle = CreateFileMapping(INVALID_HANDLE_VALUE, NULL, PAGE_READWRITE, 0, m_buf_size, NULL);
	if (!d_handle) throw std::runtime_error("circular_buffer: CreateFileMapping failed");

	LPVOID desired_base = VirtualAlloc(NULL, 2 * m_buf_size, MEM_RESERVE, PAGE_NOACCESS);
	if (!desired_base) {
		CloseHandle(d_handle);
		throw std::runtime_error("circular_buffer: VirtualAlloc reserve failed");
	}
	VirtualFree(desired_base, 0, MEM_RELEASE);

	d_first_copy = MapViewOfFileEx(d_handle, FILE_MAP_WRITE, 0, 0, m_buf_size, desired_base);
	if (d_first_copy != desired_base) {
		CloseHandle(d_handle);
		throw std::runtime_error("circular_buffer: MapViewOfFileEx (1) failed");
	}

	d_second_copy = MapViewOfFileEx(d_handle, FILE_MAP_WRITE, 0, 0, m_buf_size, (char*)desired_base + m_buf_size);
	if (d_second_copy != (char*)desired_base + m_buf_size) {
		UnmapViewOfFile(d_first_copy);
		CloseHandle(d_handle);
		throw std::runtime_error("circular_buffer: MapViewOfFileEx (2) failed");
	}

	m_buf = (char*)d_first_copy;
	m_r = 0;
	m_w = 0;
    // std::mutex does not need explicit initialization
}

circular_buffer::~circular_buffer() {
	UnmapViewOfFile(d_second_copy);
	UnmapViewOfFile(d_first_copy);
	CloseHandle(d_handle);
}

#else
// ----------------------------------------------------------------------------
// POSIX Implementation
// ----------------------------------------------------------------------------
circular_buffer::circular_buffer(unsigned int buf_len, unsigned int item_size, int overwrite) {
	if (!buf_len) throw std::runtime_error("circular_buffer: buffer len is 0");
	if (!item_size) throw std::runtime_error("circular_buffer: item size is 0");

	m_item_size = item_size;
	m_overwrite = overwrite;

	/* Check for multiplication overflow before allocating */
	if (buf_len > UINT_MAX / item_size)
		throw std::runtime_error("circular_buffer: buffer size overflow");

	long page_size = sysconf(_SC_PAGESIZE);
	if (page_size < 0) page_size = 4096;

	unsigned int raw_size = item_size * buf_len;
	m_buf_size = (raw_size + page_size - 1) & ~(page_size - 1);
	m_buf_len = m_buf_size / item_size;

	char tmp_path[] = "/tmp/kal.shm.XXXXXX";
	m_shm_fd = mkstemp(tmp_path);
	if (m_shm_fd < 0) throw std::runtime_error("circular_buffer: mkstemp failed");
	unlink(tmp_path);

	if (ftruncate(m_shm_fd, m_buf_size) < 0) {
		close(m_shm_fd);
		throw std::runtime_error("circular_buffer: ftruncate failed");
	}

	void *reserve_addr = mmap(NULL, 2 * m_buf_size, PROT_NONE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
	if (reserve_addr == MAP_FAILED) {
		close(m_shm_fd);
		throw std::runtime_error("circular_buffer: mmap reserve failed");
	}

	void *first_map = mmap(reserve_addr, m_buf_size, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_FIXED, m_shm_fd, 0);
	if (first_map != reserve_addr) {
		munmap(reserve_addr, 2 * m_buf_size);
		close(m_shm_fd);
		throw std::runtime_error("circular_buffer: mmap copy 1 failed");
	}

	void *second_map = mmap((char*)reserve_addr + m_buf_size, m_buf_size, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_FIXED, m_shm_fd, 0);
	if (second_map != (char*)reserve_addr + m_buf_size) {
		munmap(reserve_addr, 2 * m_buf_size);
		close(m_shm_fd);
		throw std::runtime_error("circular_buffer: mmap copy 2 failed");
	}

	m_buf = (char*)reserve_addr;
	m_r = 0;
	m_w = 0;
}

circular_buffer::~circular_buffer() {
	munmap(m_buf, 2 * m_buf_size);
	close(m_shm_fd);
}
#endif

// ----------------------------------------------------------------------------
// Shared Methods (using std::lock_guard)
// ----------------------------------------------------------------------------

void circular_buffer::flush() {
	std::lock_guard<std::mutex> lock(m_mutex);
	m_r = 0;
	m_w = 0;
}

unsigned int circular_buffer::data_available() {
	std::lock_guard<std::mutex> lock(m_mutex);
	unsigned int bytes_avail = m_w - m_r;
	return bytes_avail / m_item_size;
}

unsigned int circular_buffer::space_available() {
	std::lock_guard<std::mutex> lock(m_mutex);
	unsigned int bytes_avail = m_w - m_r;
	unsigned int space_bytes = m_buf_size - bytes_avail;
	return space_bytes / m_item_size;
}

unsigned int circular_buffer::capacity() {
	return m_buf_len;
}

unsigned int circular_buffer::buf_len() {
	return m_buf_len;
}

unsigned int circular_buffer::write(const void *s, unsigned int len) {
	std::lock_guard<std::mutex> lock(m_mutex);

	unsigned int bytes_used = m_w - m_r;
	unsigned int bytes_free = m_buf_size - bytes_used;
	unsigned int items_free = bytes_free / m_item_size;

	unsigned int to_write = len;
	if (!m_overwrite && to_write > items_free) {
		to_write = items_free;
	}

	if (to_write > 0) {
		unsigned int offset = m_w % m_buf_size;
		memcpy(m_buf + offset, s, to_write * m_item_size);
		m_w += to_write * m_item_size;
	}

	if (m_overwrite && (m_w - m_r) > m_buf_size) {
		m_r = m_w - m_buf_size;
	}

	if (m_r >= m_buf_size && m_w >= m_buf_size) {
		m_r -= m_buf_size;
		m_w -= m_buf_size;
	}
	return to_write;
}

unsigned int circular_buffer::read(void *s, unsigned int len) {
	std::lock_guard<std::mutex> lock(m_mutex);

	unsigned int bytes_avail = m_w - m_r;
	unsigned int items_avail = bytes_avail / m_item_size;
	unsigned int to_read = MIN(len, items_avail);

	if (to_read > 0) {
		unsigned int offset = m_r % m_buf_size;
		memcpy(s, m_buf + offset, to_read * m_item_size);
		m_r += to_read * m_item_size;
	}

	if (m_r >= m_buf_size && m_w >= m_buf_size) {
		m_r -= m_buf_size;
		m_w -= m_buf_size;
	}
	return to_read;
}

void *circular_buffer::peek(unsigned int *len) {
	std::lock_guard<std::mutex> lock(m_mutex);
	
	unsigned int bytes_avail = m_w - m_r;
	if (len) {
		*len = bytes_avail / m_item_size;
	}
	
	unsigned int offset = m_r % m_buf_size;
	return (void*)(m_buf + offset);
}

unsigned int circular_buffer::purge(unsigned int len) {
	std::lock_guard<std::mutex> lock(m_mutex);

	unsigned int bytes_avail = m_w - m_r;
	unsigned int items_avail = bytes_avail / m_item_size;
	unsigned int to_purge = MIN(len, items_avail);

	m_r += to_purge * m_item_size;

	if (m_r >= m_buf_size && m_w >= m_buf_size) {
		m_r -= m_buf_size;
		m_w -= m_buf_size;
	}
	return to_purge;
}