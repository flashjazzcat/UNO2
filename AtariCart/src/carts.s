/*

void emulate_standard_8k() {
	// 8k
	RD5_HIGH
	uint16_t addr;
	while (1)
	{
		// wait for s5 low
		while (S5_RD) ;
		SET_DATA_MODE_OUT
		// while s5 low
		while (!S5_RD) {
			addr = ADDR_IN;
			DATA_OUT = ((uint16_t)cart_ram1[addr])<<8;
		}
		SET_DATA_MODE_IN
	}
}

void emulate_standard_16k() {
	// 16k
	RD4_HIGH
	RD5_HIGH
	uint16_t addr;
	while (1)
	{
		// wait for either s4 or s5 low
		while (S4_AND_S5_HIGH) ;
		SET_DATA_MODE_OUT
		if (!S4_RD) {
			// while s4 low
			while (!S4_RD) {
				addr = ADDR_IN;
				DATA_OUT = ((uint16_t)cart_ram1[addr])<<8;
			}
		}
		else {
			// while s5 low
			while (!S5_RD) {
				addr = ADDR_IN;
				DATA_OUT = ((uint16_t)cart_ram1[0x2000|addr])<<8;
			}
		}
		SET_DATA_MODE_IN
	}
}

void emulate_XEGS_32k(char switchable) {
	// 32k
	RD4_HIGH
	RD5_HIGH
	uint16_t addr, data, c;
	unsigned char *bankPtr = &cart_ram1[0];
	while (1)
	{
		// wait for phi2 high
		while (!((c = CONTROL_IN) & PHI2)) ;

		if (!(c & S4)) {
			SET_DATA_MODE_OUT
			addr = ADDR_IN;
			DATA_OUT = ((uint16_t)(*(bankPtr+addr)))<<8;
			// wait for phi2 low
			while (CONTROL_IN & PHI2) ;
			SET_DATA_MODE_IN
		}
		else if (!(c & S5)) {
			SET_DATA_MODE_OUT
			addr = ADDR_IN;
			DATA_OUT = ((uint16_t)cart_ram1[0x6000|addr])<<8;
			// wait for phi2 low
			while (CONTROL_IN & PHI2) ;
			SET_DATA_MODE_IN
		}
		else if (!(c & CCTL) && !(c & RW)) {
			// CCTL low + write
			data = DATA_IN;
			// read the data bus on falling edge of phi2
			while (CONTROL_IN & PHI2)
				data = DATA_IN;
			// new bank is the low 2 bits written to $D5xx
			bankPtr = &cart_ram1[0] + (8192*((data>>8) & 3));
			if (switchable) {
				if (data & 0x8000) {
					RD4_LOW
					RD5_LOW
					GREEN_LED_OFF
				} else {
					RD4_HIGH
					RD5_HIGH
					GREEN_LED_ON
				}
			}
		}
	}
}

void emulate_XEGS_64k(char switchable) {
	// 64k
	RD4_HIGH
	RD5_HIGH
	uint16_t addr, data, c;
	unsigned char *bankPtr = &cart_ram1[0];
	while (1)
	{
		// wait for phi2 high
		while (!((c = CONTROL_IN) & PHI2)) ;

		if (!(c & S4)) {
			SET_DATA_MODE_OUT
			addr = ADDR_IN;
			DATA_OUT = ((uint16_t)(*(bankPtr+addr)))<<8;
			// wait for phi2 low
			while (CONTROL_IN & PHI2) ;
			SET_DATA_MODE_IN
		}
		else if (!(c & S5)) {
			SET_DATA_MODE_OUT
			addr = ADDR_IN;
			DATA_OUT = ((uint16_t)cart_ram1[0xE000|addr])<<8;
			// wait for phi2 low
			while (CONTROL_IN & PHI2) ;
			SET_DATA_MODE_IN
		}
		else if (!(c & CCTL) && !(c & RW)) {
			// CCTL low + write
			data = DATA_IN;
			// read the data bus on falling edge of phi2
			while (CONTROL_IN & PHI2)
				data = DATA_IN;
			// new bank is the low 3 bits written to $D5xx
			bankPtr = &cart_ram1[0] + (8192*((data>>8) & 7));
			if (switchable) {
				if (data & 0x8000) {
					RD4_LOW
					RD5_LOW
					GREEN_LED_OFF
				} else {
					RD4_HIGH
					RD5_HIGH
					GREEN_LED_ON
				}
			}
		}
	}
}

void emulate_XEGS_128k(char switchable) {
	// 128k
	RD4_HIGH
	RD5_HIGH
	uint16_t addr, data, c;
	unsigned char *ramPtr = &cart_ram1[0];
	while (1)
	{
		// wait for phi2 high
		while (!((c = CONTROL_IN) & PHI2)) ;

		if (!(c & S4)) {
			SET_DATA_MODE_OUT
			addr = ADDR_IN;
			DATA_OUT = ((uint16_t)(*(ramPtr+addr)))<<8;
			// wait for phi2 low
			while (CONTROL_IN & PHI2) ;
			SET_DATA_MODE_IN
		}
		else if (!(c & S5)) {
			SET_DATA_MODE_OUT
			addr = ADDR_IN;
			DATA_OUT = ((uint16_t)cart_ram2[0xE000|addr])<<8;
			// wait for phi2 low
			while (CONTROL_IN & PHI2) ;
			SET_DATA_MODE_IN
		}
		else if (!(c & CCTL) && !(c & RW)) {
			// CCTL low + write
			data = DATA_IN;
			// read the data bus on falling edge of phi2
			while (CONTROL_IN & PHI2)
				data = DATA_IN;
			// new bank is the low 4 bits written to $D5xx
			int bank = (data>>8) & 0xF;
			if (bank & 0x8) ramPtr = &cart_ram2[0]; else ramPtr = &cart_ram1[0];
			ramPtr += 8192 * (bank & 0x7);
			if (switchable) {
				if (data & 0x8000) {
					RD4_LOW
					RD5_LOW
					GREEN_LED_OFF
				} else {
					RD4_HIGH
					RD5_HIGH
					GREEN_LED_ON
				}
			}
		}
	}
}

void emulate_bounty_bob() {
	// 40k
	RD4_HIGH
	RD5_HIGH
	uint16_t addr, c;
	unsigned char *bankPtr1 = &cart_ram1[0], *bankPtr2 =  &cart_ram1[0x4000];
	while (1)
	{
		// wait for phi2 high
		while (!((c = CONTROL_IN) & PHI2)) ;
		if (!(c & S4)) {
			SET_DATA_MODE_OUT
			addr = ADDR_IN;
			if (addr & 0x1000) {
				DATA_OUT = ((uint16_t)(*(bankPtr2+(addr&0xFFF))))<<8;
				if (addr == 0x1FF6) bankPtr2 = &cart_ram1[0x4000];
				else if (addr == 0x1FF7) bankPtr2 = &cart_ram1[0x5000];
				else if (addr == 0x1FF8) bankPtr2 = &cart_ram1[0x6000];
				else if (addr == 0x1FF9) bankPtr2 = &cart_ram1[0x7000];
			}
			else {
				DATA_OUT = ((uint16_t)(*(bankPtr1+(addr&0xFFF))))<<8;
				if (addr == 0x0FF6) bankPtr1 = &cart_ram1[0];
				else if (addr == 0x0FF7) bankPtr1 = &cart_ram1[0x1000];
				else if (addr == 0x0FF8) bankPtr1 = &cart_ram1[0x2000];
				else if (addr == 0x0FF9) bankPtr1 = &cart_ram1[0x3000];
			}
		}
		else if (!(c & S5)) {
			SET_DATA_MODE_OUT
			addr = ADDR_IN;
			DATA_OUT = ((uint16_t)cart_ram1[0x8000|addr])<<8;
		}
		// wait for phi2 low
		while (CONTROL_IN & PHI2) ;
		SET_DATA_MODE_IN
	}
}

void emulate_atarimax_128k() {
	// atarimax 128k
	RD5_HIGH
	RD4_LOW
	uint16_t addr, c;
	uint32_t bank = 0;
	unsigned char *ramPtr;
	while (1)
	{
		// select the right SRAM block, based on the cartridge bank
		if (bank & 0x8) ramPtr = &cart_ram2[0]; else ramPtr = &cart_ram1[0];
		ramPtr += 8192 * (bank & 0x7);
		// wait for phi2 high
		while (!((c = CONTROL_IN) & PHI2)) ;
		if (!(c & S5)) {
			SET_DATA_MODE_OUT
			addr = ADDR_IN;
			DATA_OUT = ((uint16_t)(*(ramPtr+addr)))<<8;
		}
		else if (!(c & CCTL)) {
			// CCTL low
			addr = ADDR_IN;
			if ((addr & 0xE0) == 0) {
				bank = addr & 0xF;
				if (addr & 0x10) {
					RD5_LOW
					GREEN_LED_OFF
				}
				else {
					RD5_HIGH
					GREEN_LED_ON
				}
			}
		}
		// wait for phi2 low
		while (CONTROL_IN & PHI2) ;
		SET_DATA_MODE_IN
	}
}

void emulate_williams() {
	// williams 32k, 64k
	RD5_HIGH
	RD4_LOW
	uint16_t addr, c;
	uint32_t bank = 0;
	unsigned char *bankPtr;
	while (1)
	{
		// select the right SRAM block, based on the cartridge bank
		bankPtr = &cart_ram1[0] + (8192*bank);
		// wait for phi2 high
		while (!((c = CONTROL_IN) & PHI2)) ;
		if (!(c & S5)) {
			SET_DATA_MODE_OUT
			addr = ADDR_IN;
			DATA_OUT = ((uint16_t)(*(bankPtr+addr)))<<8;
		}
		else if (!(c & CCTL)) {
			// CCTL low
			addr = ADDR_IN;
			if ((addr & 0xF0) == 0) {
				bank = addr & 0x07;
				if (addr & 0x08) {
					RD5_LOW
					GREEN_LED_OFF
				}
				else {
					RD5_HIGH
					GREEN_LED_ON
				}
			}
		}
		// wait for phi2 low
		while (CONTROL_IN & PHI2) ;
		SET_DATA_MODE_IN
	}
}

void emulate_OSS_B() {
	// OSS type B
	RD5_HIGH
	RD4_LOW
	uint16_t addr, c;
	uint32_t bank = 1;
	unsigned char *bankPtr;
	while (1)
	{
		// select the right SRAM block, based on the cartridge bank
		bankPtr = &cart_ram1[0] + (4096*bank);
		// wait for phi2 high
		while (!((c = CONTROL_IN) & PHI2)) ;
		if (!(c & S5)) {
			SET_DATA_MODE_OUT
			addr = ADDR_IN;
			if (addr & 0x1000)
				DATA_OUT = ((uint16_t)cart_ram1[addr&0xFFF])<<8;
			else
				DATA_OUT = ((uint16_t)(*(bankPtr+addr)))<<8;
		}
		else if (!(c & CCTL)) {
			// CCTL low
			addr = ADDR_IN;
			int a0 = addr & 1, a3 = addr & 8;
			if (a3 && !a0) {
				RD5_LOW
				GREEN_LED_OFF
			}
			else {
				RD5_HIGH
				GREEN_LED_ON
				if (!a3 && !a0) bank = 1;
				else if (!a3 && a0) bank = 3;
				else if (a3 && a0) bank = 2;
			}
		}
		// wait for phi2 low
		while (CONTROL_IN & PHI2) ;
		SET_DATA_MODE_IN
	}
}

void emulate_OSS_A(char is034M) {
	// OSS type A (034M, 043M)
	RD5_HIGH
	RD4_LOW
	uint16_t addr, c;
	uint32_t bank = 0;
	unsigned char *bankPtr;
	while (1)
	{
		// select the right SRAM block, based on the cartridge bank
		bankPtr = &cart_ram1[0] + (4096*bank);
		// wait for phi2 high
		while (!((c = CONTROL_IN) & PHI2)) ;
		if (!(c & S5)) {
			SET_DATA_MODE_OUT
			addr = ADDR_IN;
			if (addr & 0x1000)
				DATA_OUT = ((uint16_t)cart_ram1[addr|0x2000])<<8;	// 4k bank #3 always mapped to $Bxxx
			else
				DATA_OUT = ((uint16_t)(*(bankPtr+addr)))<<8;
		}
		else if (!(c & CCTL)) {
			// CCTL low
			addr = ADDR_IN & 0xF;
			if (addr & 0x8) {
				RD5_LOW
				GREEN_LED_OFF
			}
			else {
				RD5_HIGH
				GREEN_LED_ON
				if (addr == 0x0) bank = 0;
				if (addr == 0x3 || addr == 0x7) bank = is034M ? 1 : 2;
				if (addr == 0x4) bank = is034M ? 2 : 1;
			}
		}
		// wait for phi2 low
		while (CONTROL_IN & PHI2) ;
		SET_DATA_MODE_IN
	}
}

void emulate_megacart(int size) {
	// 16k - 128k
	RD4_HIGH
	RD5_HIGH
	uint16_t addr, data, c;
	uint32_t bank_mask = 0x00;
	if (size == 32) bank_mask = 0x1;
	else if (size == 64) bank_mask = 0x3;
	else if (size == 128) bank_mask = 0x7;

	unsigned char *ramPtr = &cart_ram1[0];
	while (1)
	{
		// wait for phi2 high
		while (!((c = CONTROL_IN) & PHI2)) ;

		if (!(c & S4)) {
			SET_DATA_MODE_OUT
			addr = ADDR_IN;
			DATA_OUT = ((uint16_t)(*(ramPtr+addr)))<<8;
			// wait for phi2 low
			while (CONTROL_IN & PHI2) ;
			SET_DATA_MODE_IN
		}
		else if (!(c & S5)) {
			SET_DATA_MODE_OUT
			addr = ADDR_IN;
			DATA_OUT = ((uint16_t)(*(ramPtr+(addr|0x2000))))<<8;
			// wait for phi2 low
			while (CONTROL_IN & PHI2) ;
			SET_DATA_MODE_IN
		}
		else if (!(c & CCTL) && !(c & RW)) {
			// CCTL low + write
			data = DATA_IN;
			// read the data bus on falling edge of phi2
			while (CONTROL_IN & PHI2)
				data = DATA_IN;
			// new bank is the low n bits written to $D5xx
			int bank = (data>>8) & bank_mask;
			if (bank & 0x4) ramPtr = &cart_ram2[0]; else ramPtr = &cart_ram1[0];
			ramPtr += 16384 * (bank&0x3);
			if (data & 0x8000) {
				RD4_LOW
				RD5_LOW
				GREEN_LED_OFF
			} else {
				RD4_HIGH
				RD5_HIGH
				GREEN_LED_ON
			}
		}
	}
}

void emulate_SIC() {
	// 128k
	RD5_HIGH
	RD4_LOW
	uint16_t addr, data, c;
	uint8_t SIC_byte = 0;
	unsigned char *ramPtr = &cart_ram1[0];
	while (1)
	{
		// wait for phi2 high
		while (!((c = CONTROL_IN) & PHI2)) ;

		if (!(c & S4)) {
			SET_DATA_MODE_OUT
			addr = ADDR_IN;
			DATA_OUT = ((uint16_t)(*(ramPtr+addr)))<<8;
			// wait for phi2 low
			while (CONTROL_IN & PHI2) ;
			SET_DATA_MODE_IN
		}
		else if (!(c & S5)) {
			SET_DATA_MODE_OUT
			addr = ADDR_IN;
			DATA_OUT = ((uint16_t)(*(ramPtr+(addr|0x2000))))<<8;
			// wait for phi2 low
			while (CONTROL_IN & PHI2) ;
			SET_DATA_MODE_IN
		}
		else if (!(c & CCTL)) {
			// CCTL low
			addr = ADDR_IN;
			if ((addr & 0xE0) == 0) {
				if (c & RW) {
					// read from $D5xx
					SET_DATA_MODE_OUT
					DATA_OUT = ((uint16_t)SIC_byte)<<8;
					// wait for phi2 low
					while (CONTROL_IN & PHI2) ;
					SET_DATA_MODE_IN
				}
				else {
					// write to $D5xx
					data = DATA_IN;
					// read the data bus on falling edge of phi2
					while (CONTROL_IN & PHI2)
						data = DATA_IN;
					SIC_byte = (uint8_t)(data>>8);
					// switch bank
					if (SIC_byte & 0x4) ramPtr = &cart_ram2[0]; else ramPtr = &cart_ram1[0];
					ramPtr += 16384 * (SIC_byte&0x3);
					if (SIC_byte & 0x40) RD5_LOW else RD5_HIGH
					if (SIC_byte & 0x20) RD4_HIGH else RD4_LOW
					if (SIC_byte == 0x40) GREEN_LED_OFF else GREEN_LED_ON
				}
			}
		}
	}
}

void emulate_SDX(int size) {
	RD5_HIGH
	RD4_LOW
	uint16_t addr, c;
	unsigned char *ramPtr = &cart_ram1[0];
	while (1)
	{
		// wait for phi2 high
		while (!((c = CONTROL_IN) & PHI2)) ;
		if (!(c & S5)) {
			SET_DATA_MODE_OUT
			addr = ADDR_IN;
			DATA_OUT = ((uint16_t)(*(ramPtr+addr)))<<8;
		}
		else if (!(c & CCTL)) {
			// CCTL low
			addr = ADDR_IN;
			if ((addr & 0xF0) == 0xE0) {
				// 64k & 128k versions
				if (size == 64) ramPtr = &cart_ram1[0]; else ramPtr = &cart_ram2[0];
				ramPtr += ((~addr) & 0x7) * 8192;
				if (addr & 0x8) {
					RD5_LOW
					GREEN_LED_OFF
				}
				else {
					RD5_HIGH
					GREEN_LED_ON
				}
			}
			if (size == 128 && (addr & 0xF0) == 0xF0) {
				// 128k version only
				ramPtr = &cart_ram1[0] + ((~addr) & 0x7) * 8192;
				if (addr & 0x8) {
					RD5_LOW
					GREEN_LED_OFF
				}
				else {
					RD5_HIGH
					GREEN_LED_ON
				}
			}
		}
		// wait for phi2 low
		while (CONTROL_IN & PHI2) ;
		SET_DATA_MODE_IN
	}
}

void emulate_diamond_express(uint8_t cctlAddr) {
	RD5_HIGH
	RD4_LOW
	uint16_t addr, c;
	unsigned char *ramPtr = &cart_ram1[0];
	while (1)
	{
		// wait for phi2 high
		while (!((c = CONTROL_IN) & PHI2)) ;
		if (!(c & S5)) {
			SET_DATA_MODE_OUT
			addr = ADDR_IN;
			DATA_OUT = ((uint16_t)(*(ramPtr+addr)))<<8;
		}
		else if (!(c & CCTL)) {
			// CCTL low
			addr = ADDR_IN;
			if ((addr & 0xF0) == cctlAddr) {
				ramPtr = &cart_ram1[0] + ((~addr) & 0x7) * 8192;
				if (addr & 0x8) {
					RD5_LOW
					GREEN_LED_OFF
				}
				else {
					RD5_HIGH
					GREEN_LED_ON
				}
			}
		}
		// wait for phi2 low
		while (CONTROL_IN & PHI2) ;
		SET_DATA_MODE_IN
	}
}

void emulate_blizzard() {
	//16k
	RD4_HIGH
	RD5_HIGH
	uint16_t addr, c;
	while (1)
	{
		// wait for phi2 high
		while (!((c = CONTROL_IN) & PHI2)) ;

		if (!(c & S4)) {
			SET_DATA_MODE_OUT
			addr = ADDR_IN;
			DATA_OUT = ((uint16_t)cart_ram1[addr])<<8;
		}
		else if (!(c & S5)) {
			SET_DATA_MODE_OUT
			addr = ADDR_IN;
			DATA_OUT = ((uint16_t)cart_ram1[0x2000|addr])<<8;
		}
		else if (!(c & CCTL)) {
			// CCTL
			RD4_LOW
			RD5_LOW
			GREEN_LED_OFF
		}
		// wait for phi2 low
		while (CONTROL_IN & PHI2) ;
		SET_DATA_MODE_IN
	}
}

void feed_XEX_loader(void) {
	RD5_LOW
	RD4_LOW
	GREEN_LED_OFF
	uint16_t addr, data, c;
	uint32_t bank = 0;
	unsigned char *ramPtr = &cart_ram1[0];
	while (1)
	{
		// wait for phi2 high
		while (!((c = CONTROL_IN) & PHI2)) ;
		if (!(c & CCTL)) {
			// CCTL low
			if (c & RW) {
				// read
				SET_DATA_MODE_OUT
				addr = ADDR_IN & 0xFF;
				DATA_OUT = ((uint16_t)ramPtr[addr&0xFF])<<8;
				GREEN_LED_ON
			}
			else {
				// write
				addr = ADDR_IN & 0xFF;
				data = DATA_IN;
				while (CONTROL_IN & PHI2)
					data = DATA_IN;
				if (addr == 0)
					bank = (bank&0xFF00) | (data>>8);
				else if (addr == 1)
					bank = (bank&0x00FF) | (data & 0xFF00);

				if (bank & 0xFF00) ramPtr = &cart_ram2[0]; else ramPtr = &cart_ram1[0];
				ramPtr += 256 * (bank & 0x00FF);
			}
		}
		else
			GREEN_LED_OFF
		// wait for phi2 low
		while (CONTROL_IN & PHI2) ;
		SET_DATA_MODE_IN
	}
}

void emulate_cartridge(int cartType) {
	__disable_irq();	// Disable interrupts

	if (cartType == CART_TYPE_8K) emulate_standard_8k();
	else if (cartType == CART_TYPE_16K) emulate_standard_16k();
	else if (cartType == CART_TYPE_XEGS_32K) emulate_XEGS_32k(0);
	else if (cartType == CART_TYPE_XEGS_64K) emulate_XEGS_64k(0);
	else if (cartType == CART_TYPE_XEGS_128K) emulate_XEGS_128k(0);
	else if (cartType == CART_TYPE_SW_XEGS_32K) emulate_XEGS_32k(1);
	else if (cartType == CART_TYPE_SW_XEGS_64K) emulate_XEGS_64k(1);
	else if (cartType == CART_TYPE_SW_XEGS_128K) emulate_XEGS_128k(1);
	else if (cartType == CART_TYPE_BOUNTY_BOB) emulate_bounty_bob();
	else if (cartType == CART_TYPE_ATARIMAX_1MBIT) emulate_atarimax_128k();
	else if (cartType == CART_TYPE_WILLIAMS_64K) emulate_williams();
	else if (cartType == CART_TYPE_OSS_16K_TYPE_B) emulate_OSS_B();
	else if (cartType == CART_TYPE_OSS_8K) emulate_OSS_B();
	else if (cartType == CART_TYPE_OSS_16K_034M) emulate_OSS_A(1);
	else if (cartType == CART_TYPE_OSS_16K_043M) emulate_OSS_A(0);
	else if (cartType == CART_TYPE_MEGACART_16K) emulate_megacart(16);
	else if (cartType == CART_TYPE_MEGACART_32K) emulate_megacart(32);
	else if (cartType == CART_TYPE_MEGACART_64K) emulate_megacart(64);
	else if (cartType == CART_TYPE_MEGACART_128K) emulate_megacart(128);
	else if (cartType == CART_TYPE_SIC_128K) emulate_SIC();
	else if (cartType == CART_TYPE_SDX_64K) emulate_SDX(64);
	else if (cartType == CART_TYPE_SDX_128K) emulate_SDX(128);
	else if (cartType == CART_TYPE_DIAMOND_64K) emulate_diamond_express(0xD0);
	else if (cartType == CART_TYPE_EXPRESS_64K) emulate_diamond_express(0x70);
	else if (cartType == CART_TYPE_BLIZZARD_16K) emulate_blizzard();
	else if (cartType == CART_TYPE_XEX) feed_XEX_loader();
	else
	{	// no cartridge (cartType = 0)
		GREEN_LED_OFF
		RD4_LOW
		RD5_LOW
		while (1) ;
	}
}
*/
