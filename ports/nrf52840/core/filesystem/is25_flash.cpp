#include <algorithm>
#include <vector>
#include <iomanip>
#include "IS25LP128F.hpp"
#include "bsp.hpp"
#include "debug.hpp"
#include "is25_flash.hpp"

static constexpr uint8_t dummy_verify_value = 0xAA;

void Is25Flash::init()
{
	// Initialise the IS25LP128F flash chip and set it up for QSPI

	nrf_qspi_cinstr_conf_t config;
	uint8_t status;
	uint8_t rx_buffer[3];
	uint8_t tx_buffer[1];

	nrfx_qspi_init(&BSP::QSPI_Inits[BSP::QSPI_0].config, nullptr, nullptr);

    config.io2_level = false;
	// Keep IO3 high during transfers as this is the reset line in SPI mode
	// We'll make use of this line once the FLASH chip is in QSPI mode
    config.io3_level = true;
    config.wipwait   = false;

	// Read and check the SPI device ID matches the expected value
	config.opcode = IS25LP128F::RDJDID;
	config.length = NRF_QSPI_CINSTR_LEN_4B;
	config.wren = false;

	nrfx_qspi_cinstr_xfer(&config, nullptr, rx_buffer);

    if (rx_buffer[0] != IS25LP128F::MANUFACTURER_ID ||
        rx_buffer[1] != IS25LP128F::MEMORY_TYPE_ID ||
        rx_buffer[2] != IS25LP128F::CAPACITY_ID)
    {
		DEBUG_ERROR("IS25LP128F not correctly identified");
        return;
    }

	// Set FLASH output drive to 12.5%
    config.opcode = IS25LP128F::SERPV;
    tx_buffer[0] = 1 << 5;
	config.length = NRF_QSPI_CINSTR_LEN_2B;
	config.wren = true;
    nrfx_qspi_cinstr_xfer(&config, tx_buffer, nullptr);

	// Switch to QSPI mode
	config.opcode = IS25LP128F::WRSR;
    tx_buffer[0] = IS25LP128F::STATUS_QE;
	config.length = NRF_QSPI_CINSTR_LEN_2B;
	config.wren = true;
	nrfx_qspi_cinstr_xfer(&config, tx_buffer, nullptr);

	// Wait for QSPI to be programmed
	config.opcode = IS25LP128F::RDSR;
	config.length = NRF_QSPI_CINSTR_LEN_2B;
	config.wren = false;
    do
    {
		nrfx_qspi_cinstr_xfer(&config, nullptr, rx_buffer);
		status = rx_buffer[0];		
    }
	while (status & IS25LP128F::STATUS_WIP);
}

// The maximum read size is 0x3FFFF, size must be a multiple of 4, buffer must be word aligned
int Is25Flash::read(lfs_block_t block, lfs_off_t off, void * buffer, lfs_size_t size)
{
	//DEBUG_TRACE("QSPI Flash read(%lu %lu %lu)", block, off, size);
	nrfx_err_t ret = nrfx_qspi_read(buffer, size, block * m_block_size + off);
	if (ret != NRFX_SUCCESS)
	{
		DEBUG_ERROR("QSPI IO Error %d", ret);
		return LFS_ERR_IO;
	}

	return LFS_ERR_OK;
}

// The maximum program size is 0x3FFFF, size must be a multiple of 4, buffer must be word aligned
int Is25Flash::prog(lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size)
{
	//DEBUG_TRACE("QSPI Flash prog(%lu %lu %lu)", block, off, size);

	nrfx_err_t ret_write = nrfx_qspi_write(buffer, size, block * m_block_size + off);
	if (ret_write != NRFX_SUCCESS)
	{
		DEBUG_ERROR("QSPI IO Error %d", ret_write);
		return LFS_ERR_IO;
	}

	// Wait for the write to be completed before verifying it
	int ret_sync = sync();
	if (ret_sync)
		return ret_sync;

	// Check that all bytes were written correctly
	std::vector<uint8_t> read_buffer;
	read_buffer.resize(size, dummy_verify_value);

	read(block, off, &read_buffer[0], read_buffer.size());

	if (memcmp( reinterpret_cast<const uint8_t *>(buffer), &read_buffer[0], size ))
	{
		DEBUG_ERROR("QSPI Flash prog reported a bad write");
		for (unsigned int i = 0; i < size; i++)
			std::cout << std::setfill('0') << std::setw(2) << std::hex << (((unsigned int)((uint8_t *)buffer)[i]) & 0xFF);
		std::cout << std::endl;
		for (unsigned int i = 0; i < size; i++)
			std::cout << std::setfill('0') << std::setw(2) << std::hex << (((unsigned int)read_buffer[i]) & 0xFF);
		std::cout << std::endl;
		return LFS_ERR_CORRUPT;
	}
	
	return LFS_ERR_OK;
}

int Is25Flash::erase(lfs_block_t block)
{
	//DEBUG_TRACE("QSPI Flash erase(%lu)", block);

	nrfx_err_t ret_erase = nrfx_qspi_erase(NRF_QSPI_ERASE_LEN_4KB, block * m_block_size);
	if (ret_erase != NRFX_SUCCESS)
	{
		DEBUG_ERROR("QSPI IO Error %d", ret_erase);
		return LFS_ERR_IO;
	}

	// Wait for the erase to be completed before verifying it
	int ret_sync = sync();
	if (ret_sync)
		return ret_sync;
	
	// Check the block erased correctly by reading it back
	std::vector<uint8_t> read_buffer;
	read_buffer.resize(m_block_size, dummy_verify_value);

	int read_ret = read(block, 0, &read_buffer[0], read_buffer.size());
	if (read_ret)
		return read_ret;

	// Check all bytes were erased correctly
	if (std::any_of(read_buffer.cbegin(), read_buffer.cend(), [](uint8_t i){ return i != 0xFF; }))
	{
		DEBUG_ERROR("QSPI Flash erase failed to erase");
		return LFS_ERR_CORRUPT;
	}

	return LFS_ERR_OK;
}

int Is25Flash::sync()
{
	//DEBUG_TRACE("QSPI Sync()");
	nrfx_err_t ret;
	do
	{
		ret = nrfx_qspi_mem_busy_check();
	}
	while (ret == NRFX_ERROR_BUSY);

	if (ret != NRFX_SUCCESS)
	{
		DEBUG_ERROR("QSPI IO Sync %d", ret);
		return LFS_ERR_IO;
	}

	return LFS_ERR_OK;
}
