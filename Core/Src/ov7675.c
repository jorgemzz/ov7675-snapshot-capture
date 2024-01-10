/*
 * ov7675.c
 *
 *  Created on: Jan 9, 2024
 *      Author: jorgemiranda
 */

#include <stdio.h>
#include <ov7675.h>

#include "ov7670Reg.h"

I2C_HandleTypeDef *p_hi2c = NULL;

int info_clock_speed = 16;	/* External clock speed (MHz) */
uint8_t info_clkrc = 1;		/* Clock divider value */
uint8_t info_pll_bypass = 0;
uint8_t info_pclk_hb_disable = 1;
uint8_t info_fps = 1;

struct ov7670_fract {
	uint32_t numerator;
	uint32_t denominator;
};

void Error_Handler(void);

uint8_t ov7675_i2c_write_register(uint8_t reg_addr, uint8_t data);
uint8_t ov7675_i2c_read_register(uint8_t reg_addr, uint8_t *data);
uint8_t ov7675_write_array(struct regval_list *vals);

void ov7675_apply_fmt();
void ov7675_apply_framerate();
void ov7675_set_framerate(struct ov7670_fract *tpf);
void ov7675_get_framerate(struct ov7670_fract *tpf);
uint8_t ov7675_set_hw(int hstart, int hstop, int vstart, int vstop);

void ov7675_init(I2C_HandleTypeDef *phi2c){
	p_hi2c = phi2c;
	ov7675_i2c_write_register(0x12, 0x80);  // RESET
	printf("ov7675_init: reset\n\r");
	HAL_Delay(30);

	uint8_t buffer[4];
	ov7675_i2c_read_register(0x0b, buffer);
	printf("ov7675_init: dev id = 0x%02X\n\r", buffer[0]);
}

void ov7675_config(){
	printf("ov7675_config: write array default regs\n\r");
	ov7675_write_array(ov7670_default_regs);

	/*for(int i = 0; OV7670_reg[i][0] != REG_BATT; i++) {
		ov7675_i2c_write_register(OV7670_reg[i][0], OV7670_reg[i][1]);
		HAL_Delay(1);
	}*/

	printf("ov7675_config: apply format\n\r");
	ov7675_apply_fmt();

	printf("ov7675_config: apply framerate\n\r");
	ov7675_apply_framerate();

	printf("ov7675_config: set framerate\n\r");
	struct ov7670_fract tpf;
	tpf.numerator = 1;
	tpf.denominator = info_fps;
	ov7675_set_framerate(&tpf);

	printf("ov7675_config: done\n\r");
}

uint8_t ov7675_i2c_write_register(uint8_t reg_addr, uint8_t data){
	HAL_StatusTypeDef ret;
	ret = HAL_I2C_Mem_Write(p_hi2c, OV7675_I2C_SLAVE_ADDR, reg_addr, 1, &data, 1, HAL_MAX_DELAY);
	if(ret != HAL_OK){
		Error_Handler();
	}
	return 0;
}

uint8_t ov7675_i2c_read_register(uint8_t reg_addr, uint8_t *data){
	HAL_StatusTypeDef ret;
	//ret = HAL_I2C_Mem_Read(p_hi2c, OV7675_I2C_SLAVE_ADDR, reg_addr, 1, (unsigned char*)data, 1, HAL_MAX_DELAY);
	ret = HAL_I2C_Master_Transmit(p_hi2c, OV7675_I2C_SLAVE_ADDR, &reg_addr, 1, HAL_MAX_DELAY);
	ret |= HAL_I2C_Master_Receive(p_hi2c, OV7675_I2C_SLAVE_ADDR, data, 1, HAL_MAX_DELAY);
	if(ret != HAL_OK){
		Error_Handler();
	}
	return 0;
}

/*
 * Write a list of register settings; ff/ff stops the process.
 */
uint8_t ov7675_write_array(struct regval_list *vals){
	while (vals->reg_num != 0xff || vals->value != 0xff){
		uint8_t ret = ov7675_i2c_write_register(vals->reg_num, vals->value);
		if(ret != 0){
			Error_Handler();
		}
		vals++;
		HAL_Delay(1);
	}
	return 0;
}

void ov7675_apply_fmt(){
	unsigned char com7, com10 = 0;
	com7 = ov7670_fmt_rgb565[0].value;
	com7 |= ov7675_win_sizes[QCIF].com7_bit;

	//ret = ov7670_write(sd, REG_COM7, com7);
	ov7675_i2c_write_register(REG_COM7, com7);

	if (info_pclk_hb_disable)
		com10 |= COM10_PCLK_HB;

	//ret = ov7670_write(sd, REG_COM10, com10);
	ov7675_i2c_write_register(REG_COM10, com10);

	//ret = ov7670_write_array(sd, info->fmt->regs + 1);
	ov7675_write_array(ov7670_fmt_rgb565 + 1);

	//ret = ov7670_set_hw(sd, wsize->hstart, wsize->hstop, wsize->vstart, wsize->vstop);
	ov7675_set_hw(ov7675_win_sizes[QCIF].hstart, ov7675_win_sizes[QCIF].hstop,
			ov7675_win_sizes[QCIF].vstart, ov7675_win_sizes[QCIF].vstop);


	//ret = ov7670_write_array(sd, wsize->regs);
	ov7675_write_array(ov7670_qcif_regs);

	//ret = ov7670_write(sd, REG_CLKRC, info->clkrc);
	//no encontre detalle
	ov7675_i2c_write_register(REG_CLKRC, info_clkrc);
}

void ov7675_apply_framerate(){
	ov7675_i2c_write_register(REG_CLKRC, info_clkrc);

	ov7675_i2c_write_register(REG_DBLV, info_pll_bypass ? DBLV_BYPASS : DBLV_X4);
}

void ov7675_set_framerate(struct ov7670_fract *tpf){
	uint32_t clkrc;
	int pll_factor;

	/*
	 * The formula is fps = 5/4*pixclk for YUV/RGB and
	 * fps = 5/2*pixclk for RAW.
	 *
	 * pixclk = clock_speed / (clkrc + 1) * PLLfactor
	 *
	 */
	if (tpf->numerator == 0 || tpf->denominator == 0) {
		clkrc = 0;
	} else {
		pll_factor = info_pll_bypass ? 1 : PLL_FACTOR;
		clkrc = (5 * pll_factor * info_clock_speed * tpf->numerator) /
			(4 * tpf->denominator);
		clkrc--;
	}

	/*
	 * The datasheet claims that clkrc = 0 will divide the input clock by 1
	 * but we've checked with an oscilloscope that it divides by 2 instead.
	 * So, if clkrc = 0 just bypass the divider.
	 */
	if (clkrc <= 0)
		clkrc = CLK_EXT;
	else if (clkrc > CLK_SCALE)
		clkrc = CLK_SCALE;
	info_clkrc = clkrc;

	/* Recalculate frame rate */
	ov7675_get_framerate(tpf);

	ov7675_apply_framerate();
}

void ov7675_get_framerate(struct ov7670_fract *tpf){
	uint32_t clkrc = info_clkrc;
	int pll_factor;

	if (info_pll_bypass)
		pll_factor = 1;
	else
		pll_factor = PLL_FACTOR;

	clkrc++;

	tpf->numerator = 1;
	tpf->denominator = (5 * pll_factor * info_clock_speed) /
			(4 * clkrc);
}

/*
 * Store a set of start/stop values into the camera.
 */
uint8_t ov7675_set_hw(int hstart, int hstop, int vstart, int vstop){
	uint8_t ret;
	unsigned char v;
/*
 * Horizontal: 11 bits, top 8 live in hstart and hstop.  Bottom 3 of
 * hstart are in href[2:0], bottom 3 of hstop in href[5:3].  There is
 * a mystery "edge offset" value in the top two bits of href.
 */
	ret =  ov7675_i2c_write_register(REG_HSTART, (hstart >> 3) & 0xff);
	ret += ov7675_i2c_write_register(REG_HSTOP, (hstop >> 3) & 0xff);
	ret += ov7675_i2c_read_register(REG_HREF, &v);
	v = (v & 0xc0) | ((hstop & 0x7) << 3) | (hstart & 0x7);
	HAL_Delay(10);
	ret += ov7675_i2c_write_register(REG_HREF, v);
/*
 * Vertical: similar arrangement, but only 10 bits.
 */
	ret += ov7675_i2c_write_register(REG_VSTART, (vstart >> 2) & 0xff);
	ret += ov7675_i2c_write_register(REG_VSTOP, (vstop >> 2) & 0xff);
	ret += ov7675_i2c_read_register(REG_VREF, &v);
	v = (v & 0xf0) | ((vstop & 0x3) << 2) | (vstart & 0x3);
	HAL_Delay(10);
	ret += ov7675_i2c_write_register(REG_VREF, v);
	return ret;
}
