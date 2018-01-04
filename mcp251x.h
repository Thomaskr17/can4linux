#ifndef __MCP251X_H__
#define __MCP251X_H__




struct mcp251x_platform_data {
	unsigned long oscillator_frequency;
	int (*board_specific_setup)(struct spi_device *spi);
	int (*device_reset)(struct spi_device *spi);
	int (*transceiver_enable)(int enable);
};

#endif /* __MCP251X_H__ */
